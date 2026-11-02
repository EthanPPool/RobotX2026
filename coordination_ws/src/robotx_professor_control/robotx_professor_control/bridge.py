"""Calibrated force backend inheriting the existing vehicle authority boundary.

Requires the supplied publish_autonomy_command hook in boat_vehicle. Default
hardware_output_enabled=False; enabling is not an arming or mode-switch action.
"""
import json
import math
import rclpy
from std_msgs.msg import String
from mavros_msgs.msg import ManualControl
from robotx_control_interfaces.msg import ThrusterForces
from boat_vehicle.mavros_command_bridge import MavrosCommandBridge
from robotx_coordination.geometry import reference_id
from .actuation import Calibration
from .protocol import fresh
from .node import seconds


class ProfessorForceBridge(MavrosCommandBridge):
    def __init__(self):
        if not hasattr(MavrosCommandBridge,'publish_autonomy_command'):
            raise RuntimeError('Install the vehicle backend hook before launching this bridge')
        super().__init__()
        p=lambda n,d:self.declare_parameter(n,d).value
        self.hardware_enabled=bool(p('hardware_output_enabled',False))
        self.contract_confirmed=bool(p('deployment_contract_confirmed',False))
        self.expected_frame=str(p('map_frame','robotx_map'))
        self.expected_ref=reference_id(float(p('origin_latitude',0.)),float(p('origin_longitude',0.)),float(p('water_altitude',0.)))
        self.force_timeout=float(p('force_timeout',.15));self.state_timeout=float(p('state_timeout',.5))
        if not all(math.isfinite(x) and x>0 for x in (self.force_timeout,self.state_timeout)):raise ValueError('INVALID_FORCE_TIMEOUTS')
        if self.allowed_modes!=['MANUAL']:raise ValueError('FORCE_BACKEND_REQUIRES_EXPLICIT_MANUAL_MODE_CONFIGURATION')
        self.calibration=None;self.calibration_error='CALIBRATION_FILE_NOT_CONFIGURED'
        path=str(p('calibration_file',''))
        if path:
            try:self.calibration=Calibration.load(path);self.calibration_error=''
            except (ValueError,KeyError,TypeError,OSError) as exc:self.calibration_error=str(exc)
        self.last_force_seq=0;self.controller_id='';self.retired_controllers=set()
        self.state_received_at=0.;self.force_reason='WAITING_FOR_FORCE_COMMAND'
        self.force_status=self.create_publisher(String,'/vehicle/professor_actuation_status',10)
        self.create_subscription(ThrusterForces,'/control/thruster_forces',self.force_callback,10)

    def now_seconds(self):return self.get_clock().now().nanoseconds/1e9

    def command_callback(self,msg):
        # Legacy cmd_vel cannot acquire the force backend's command channel.
        pass

    def state_callback(self,msg):
        self.state_received_at=self.now_seconds()
        super().state_callback(msg)

    def publish_velocity_neutral(self):self.publish_manual_neutral()

    def command_is_zero(self,msg):
        if hasattr(msg,'twist'):return super().command_is_zero(msg)
        return abs(msg.right_force_n)<=1e-9 and abs(msg.left_force_n)<=1e-9

    def command_is_finite(self,msg):
        if hasattr(msg,'twist'):return super().command_is_finite(msg)
        return all(math.isfinite(v) for v in (msg.right_force_n,msg.left_force_n))

    def validate_force(self,msg):
        now=self.now_seconds()
        if not self.hardware_enabled:raise ValueError('HARDWARE_OUTPUT_DISABLED')
        if not self.contract_confirmed or not msg.model_contract_confirmed:raise ValueError('MODEL_DEPLOYMENT_CONTRACT_UNCONFIRMED')
        if self.calibration is None:raise ValueError(self.calibration_error)
        if not msg.valid or not msg.run_id or not msg.controller_id:raise ValueError('FORCE_REFERENCE_INVALID')
        if msg.header.frame_id!=self.expected_frame or msg.reference_id!=self.expected_ref:raise ValueError('FORCE_DATUM_MISMATCH')
        if not self.command_is_finite(msg) or not fresh(seconds(msg.header.stamp),now,self.force_timeout):raise ValueError('FORCE_COMMAND_NONFINITE_STALE_OR_FUTURE')
        if not now<seconds(msg.valid_until)<=seconds(msg.header.stamp)+self.force_timeout+1e-8:
            raise ValueError('FORCE_VALIDITY_WINDOW_EXPIRED_OR_EXCESSIVE')
        if not fresh(self.state_received_at,now,self.state_timeout):raise ValueError('VEHICLE_STATE_STALE')
        allowed,reason=self.battery_propulsion_allowed()
        if not allowed or not self.battery_is_fresh():raise ValueError('BATTERY_UNAVAILABLE_'+reason)
        return self.calibration.map(msg.right_force_n,msg.left_force_n,self.battery_voltage)

    def force_callback(self,msg):
        try:
            self.validate_force(msg)
            if msg.controller_id!=self.controller_id:
                if msg.controller_id in self.retired_controllers:raise ValueError('RETIRED_CONTROLLER_REPLAY')
                if self.controller_id and bool(self.get_parameter('autonomy_enabled').value):raise ValueError('CONTROLLER_RESTART_REQUIRES_REENABLE')
                if self.controller_id:self.retired_controllers.add(self.controller_id)
                self.controller_id=msg.controller_id;self.last_force_seq=0
            if msg.sequence<=self.last_force_seq:raise ValueError('FORCE_SEQUENCE_REPLAY')
            self.last_force_seq=msg.sequence
            self.last_command=msg;self.last_command_time=self.get_clock().now()
            self.force_reason='CALIBRATED_FORCE_READY'
        except (ValueError,TypeError,KeyError) as exc:
            self.force_reason=str(exc);self.clear_stored_command()
            if bool(self.get_parameter('autonomy_enabled').value):self.trip_to_hold(self.force_reason)

    def autonomy_callback(self,request,response):
        if request.data and (not self.hardware_enabled or not self.contract_confirmed or self.calibration is None):
            response.success=False;response.message='Force backend needs deployment contract, measured calibration and hardware_output_enabled'
            return response
        response=super().autonomy_callback(request,response)
        if not response.success and 'is not GUIDED' in response.message:
            response.message=response.message.replace('is not GUIDED','is not MANUAL (the configured force backend mode)')
        return response

    def publish_autonomy_command(self):
        try:
            values=self.validate_force(self.last_command)
            # Keep the project's validated Rover MANUAL_CONTROL axis mapping:
            # y=steering, z=throttle; neutral y=z=0. No RC channel numbers assumed.
            out=ManualControl();out.x=out.r=0.;out.buttons=0
            out.y=float(1000*values['steering']);out.z=float(1000*values['throttle'])
            self.manual_control_pub.publish(out)
            self.force_reason='FORCE_OUTPUT_AUTHORIZED'
            self.publish_force_status(values)
        except (ValueError,TypeError,KeyError,AttributeError) as exc:
            self.force_reason=str(exc);self.publish_manual_neutral();self.trip_to_hold(self.force_reason)

    def publish_force_status(self,values=None):
        msg=String();msg.data=json.dumps(dict(interface='calibrated_force_manual',reason=self.force_reason,
            hardware_enabled=self.hardware_enabled,calibrated=self.calibration is not None,
            model_contract_confirmed=self.contract_confirmed,actuation=values),allow_nan=False)
        self.force_status.publish(msg)

    def publish_control_diagnostics(self):
        # Avoid labeling force inputs as m/s in the legacy Twist diagnostics.
        if not hasattr(self,'force_status'):return
        self.publish_force_status()
        state=self.vehicle_state;age=self.command_age();reason=self.diagnostic_bridge_reason()
        data=dict(control_interface='calibrated_force_manual',bridge_reason=reason,
            bridge_autonomy_enabled=bool(self.get_parameter('autonomy_enabled').value),
            software_estop=bool(self.get_parameter('software_estop').value),
            bridge_connected=bool(state and state.connected),bridge_armed=bool(state and state.armed),
            bridge_mode=state.mode if state else 'UNKNOWN',bridge_input_age_s=age,
            bridge_command_fresh=bool(self.last_command and age is not None and age<=self.deadman_timeout),
            bridge_command_zero=self.command_is_zero(self.last_command) if self.last_command else None,
            bridge_output_authorized=reason in ('AUTHORIZED','AUTHORIZED_ZERO'),
            bridge_input_linear_x=None,bridge_input_angular_z=None,
            right_force_n=self.last_command.right_force_n if self.last_command else None,
            left_force_n=self.last_command.left_force_n if self.last_command else None,
            hold_required=self.hold_required,last_stop_reason=self.last_stop_reason,
            operator_requested=self.operator_requested,operator_active=self.operator_active,
            operator_session_owned=self.operator_session_owned)
        msg=String();msg.data=json.dumps(data,allow_nan=False);self.control_diagnostics_pub.publish(msg)

    def update(self):
        # Reuse the complete inherited software-stop/operator/mode/deadman path.
        # Battery checks remain enforced for this calibrated output backend.
        self.evaluate_battery_safety()
        if (bool(self.get_parameter('autonomy_enabled').value) and
                not fresh(self.state_received_at,self.now_seconds(),getattr(self,'state_timeout',.5))):
            self.trip_to_hold('VEHICLE_STATE_STALE')
        super().update()


def main(args=None):
    rclpy.init(args=args);node=ProfessorForceBridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.shutdown_to_hold();node.destroy_node();rclpy.shutdown()
