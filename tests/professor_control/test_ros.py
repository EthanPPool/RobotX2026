from dataclasses import replace
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from robotx_coordination.geometry import reference_id
from robotx_professor_control.tracking import Motion
from ros_stubs import FakeNode,Now,Message,Twist,header,stamp,load_nodes
from test_actuation import calibration_data

ROOT=Path(__file__).resolve().parents[2]
NODE,BRIDGE,BASE,MODS=load_nodes(ROOT)


def odom(t,x=0.,y=0.,yaw=0.):
    return NS(header=header(t),child_frame_id='boat/base_link',
              pose=NS(pose=NS(position=NS(x=x,y=y,z=0.),orientation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2))),covariance=[.01]*36),
              twist=NS(twist=Twist().twist))


def preview(t,run='run',valid=True,phase='TRANSIT'):
    return NS(header=header(t),reference_id=reference_id(0.,0.,0.),run_id=run,phase=phase,valid=valid,dt=.1,
              samples=[NS(offset_seconds=i*.1,x=0.,y=0.,yaw=0.,vx=0.,vy=0.,ax=0.,ay=0.,yaw_rate=0.,yaw_acceleration=0.) for i in range(3)])


class ControllerTests(unittest.TestCase):
    def make(self,**params):
        with patch.object(FakeNode,'overrides',dict(datum_configured=True,derivative_filter_tau=0.,**params)):
            n=NODE.ProfessorController()
        for t in (10.,10.05,10.1):n.time=t;n.on_pose(odom(t))
        n.on_reference(preview(10.1));return n

    def output(self,n):return n.pub['/control/thruster_forces'][-1]

    def test_ros_shadow_preserves_unconfirmed_contract_and_uses_three_pose_history(self):
        n=self.make();n.tick();out=self.output(n)
        self.assertTrue(out.valid);self.assertFalse(out.model_contract_confirmed)
        self.assertEqual(out.right_force_n,0.);self.assertTrue(out.controller_id)
        self.assertNotIn('/control/cmd_vel',n.pub)
        self.assertEqual(n.pub['/control/professor_status'][-1].data.count('causal_continuous'),1)

    def test_stale_pose_reference_hold_invalid_or_wrong_frame_zeros_and_resets(self):
        for cause in ('stale','hold','invalid','frame','datum','replay'):
            with self.subTest(cause=cause):
                n=self.make();n.integrator.forces=(1.,-1.)
                if cause=='stale':n.time=10.5
                else:
                    n.time=10.15;p=preview(10.15)
                    if cause=='hold':p.phase='HOLD'
                    if cause=='invalid':p.valid=False
                    if cause=='frame':p.header.frame_id='other'
                    if cause=='datum':p.reference_id='other'
                    if cause=='replay':p.header.stamp=stamp(10.1)
                    n.on_reference(p)
                n.tick();out=self.output(n)
                self.assertFalse(out.valid);self.assertEqual((out.right_force_n,out.left_force_n),(0.,0.))
                self.assertEqual(n.integrator.forces,(0.,0.))

    def test_large_tracking_error_is_rejected_and_cannot_integrate(self):
        n=self.make();n.motion=replace(n.motion,y=2.);n.tick()
        self.assertFalse(self.output(n).valid);self.assertIn('LOCAL_MODEL',self.output(n).reason)

    def test_new_mission_resets_integrated_forces(self):
        n=self.make();n.tick();n.integrator.forces=(2.,3.)
        n.time=10.15;n.on_pose(odom(10.15));n.on_reference(preview(10.15,run='new-run'));n.tick()
        self.assertEqual(self.output(n).run_id,'new-run')
        self.assertEqual((self.output(n).right_force_n,self.output(n).left_force_n),(0.,0.))

    def test_pose_only_shared_adapter_empty_twist_does_not_hide_motion(self):
        with patch.object(FakeNode,'overrides',dict(datum_configured=True,derivative_filter_tau=0.)):
            n=NODE.ProfessorController()
        for t in (10.,10.05,10.1):n.time=t;n.on_pose(odom(t,x=.2*(t-10)))
        self.assertAlmostEqual(n.motion.vx,.2)
        self.assertEqual(n.motion.vy,0.)

    def test_reference_time_skew_is_bounded(self):
        n=self.make();n.time=10.19;n.on_reference(preview(10.19));n.tick()
        self.assertFalse(self.output(n).valid);self.assertIn('TIME_MISMATCH',self.output(n).reason)

    def test_force_expiry_cannot_outlive_pose_reference_or_control_deadman(self):
        n=self.make();n.tick();out=self.output(n)
        expiry=out.valid_until.sec+out.valid_until.nanosec/1e9
        self.assertLessEqual(expiry,n.motion.stamp+n.pose_timeout)
        self.assertLessEqual(expiry,n.preview.stamp+n.reference_timeout)
        self.assertLessEqual(expiry,n.time+2*n.period+1e-9)


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        path=Path(self.tmp.name)/'calibration.json';path.write_text(json.dumps(calibration_data()))
        with patch.object(FakeNode,'overrides',dict(allowed_modes=['MANUAL'],hardware_output_enabled=True,
            deployment_contract_confirmed=True,calibration_file=str(path),battery_required_for_propulsion=True)):
            self.n=BRIDGE.ProfessorForceBridge()
        self.n.state_callback(NS(connected=True,armed=False,mode='MANUAL'))
        self.n.battery_callback(NS(voltage=15.))

    def force(self,**changes):
        msg=NS(header=header(self.n.time),valid_until=stamp(self.n.time+.1),reference_id=self.n.expected_ref,run_id='run',controller_id='controller',
               sequence=1,valid=True,model_contract_confirmed=True,right_force_n=5.,left_force_n=5.)
        msg.__dict__.update(changes);return msg

    def enable(self):
        self.n.params['software_estop']=False
        result=self.n.autonomy_callback(NS(data=True),NS())
        self.assertTrue(result.success)
        self.n.force_callback(self.force())

    def last(self):return self.n.pub['/mavros/manual_control/send'][-1]

    def test_default_software_stop_prevents_output_and_legacy_twist_cannot_control(self):
        self.n.force_callback(self.force());self.n.update()
        self.assertEqual((self.last().y,self.last().z),(0.,0.))
        self.n.clear_stored_command();self.n.command_callback(Twist());self.assertIsNone(self.n.last_command)

    def test_authorized_calibrated_manual_output_uses_existing_rover_y_z_mapping(self):
        self.enable();self.n.update()
        self.assertEqual(self.last().z,750.);self.assertEqual(self.last().y,250.)
        self.assertEqual(self.last().x,0.);self.assertEqual(self.last().r,0.)
        self.assertEqual(self.n.pub['/mavros/setpoint_velocity/cmd_vel'],[])

    def test_diagnostics_keep_authority_fields_without_labeling_forces_as_velocity(self):
        self.enable();self.n.update()
        data=json.loads(self.n.pub['/vehicle/control_diagnostics'][-1].data)
        self.assertEqual(data['control_interface'],'calibrated_force_manual')
        self.assertTrue(data['bridge_autonomy_enabled']);self.assertEqual(data['bridge_mode'],'MANUAL')
        self.assertIsNone(data['bridge_input_linear_x']);self.assertEqual(data['right_force_n'],5.)

    def test_autonomy_stop_preserves_inherited_hold_and_neutral(self):
        self.enable();self.n.update();self.n.autonomy_callback(NS(data=False),NS())
        self.assertFalse(self.n.params['autonomy_enabled'])
        self.assertEqual((self.last().y,self.last().z),(0.,0.))
        self.assertTrue(self.n.hold_required)

    def test_bad_force_stale_state_or_battery_never_produces_thrust(self):
        for failure in ('invalid','nonfinite','contract','frame','future','expired','range','battery','state','hardware'):
            with self.subTest(failure=failure):
                # Restore independent authority/session between each fault.
                self.setUp();self.enable()
                msg=self.force(sequence=2)
                if failure=='invalid':msg.valid=False
                if failure=='nonfinite':msg.right_force_n=float('nan')
                if failure=='contract':msg.model_contract_confirmed=False
                if failure=='frame':msg.header.frame_id='other'
                if failure=='future':msg.header.stamp=stamp(20.)
                if failure=='expired':msg.valid_until=stamp(9.)
                if failure=='range':msg.right_force_n=50.
                if failure=='battery':self.n.battery_voltage=12.
                if failure=='state':self.n.state_received_at=1.
                if failure=='hardware':self.n.hardware_enabled=False
                with patch.dict('sys.modules',MODS):self.n.force_callback(msg);self.n.update()
                self.assertFalse(self.n.params['autonomy_enabled'])
                self.assertEqual((self.last().y,self.last().z),(0.,0.))

    def test_controller_restart_requires_reenable_and_old_instance_cannot_resume(self):
        self.enable();self.n.force_callback(self.force(controller_id='new-controller',sequence=1))
        self.assertFalse(self.n.params['autonomy_enabled'])
        self.n.force_callback(self.force(controller_id='new-controller',sequence=2))
        self.assertEqual(self.n.controller_id,'new-controller')
        self.n.force_callback(self.force(controller_id='controller',sequence=3))
        self.assertIn('RETIRED',self.n.force_reason)

    def test_sequence_replay_and_missing_new_messages_trip_neutral(self):
        self.enable();self.n.force_callback(self.force())
        self.assertFalse(self.n.params['autonomy_enabled'])
        self.assertIn('REPLAY',self.n.force_reason)
        self.setUp();self.enable();self.n.time+=.2
        self.n.update();self.assertFalse(self.n.params['autonomy_enabled'])
        self.assertEqual((self.last().y,self.last().z),(0.,0.))

    def test_operator_takeover_has_priority_over_force_controller(self):
        self.enable();n=self.n
        n.operator_requested=n.operator_active=n.operator_session_owned=n.operator_deadman=True
        n.operator_deadman_time=n.operator_command_time=n.get_clock().now()
        n.operator_command=Twist();n.operator_command.twist.linear.x=.1
        n.update()
        self.assertAlmostEqual(self.last().z,.1/n.max_forward_speed*n.operator_manual_axis_max)
        self.assertNotEqual(self.last().z,750.)

    def test_wrong_mode_or_disconnection_preserves_inherited_neutral_stop(self):
        for mode,connected in (('GUIDED',True),('MANUAL',False)):
            self.setUp();self.enable();self.n.state_callback(NS(connected=connected,armed=False,mode=mode));self.n.update()
            self.assertFalse(self.n.params['autonomy_enabled'])
            self.assertEqual((self.last().y,self.last().z),(0.,0.))

    def test_ros_reference_to_force_to_calibrated_actuation_pipeline(self):
        with patch.object(FakeNode,'overrides',dict(datum_configured=True,derivative_filter_tau=0.,
            units_confirmed_si=True,axis_binding_confirmed=True,nominal_trim_confirmed=True)):
            controller=NODE.ProfessorController()
        for t in (10.,10.05,10.1):
            controller.time=t;controller.on_pose(odom(t,x=.05))
        controller.on_reference(preview(10.1));controller.tick()
        command=controller.pub['/control/thruster_forces'][-1]
        self.assertTrue(command.valid);self.assertTrue(command.model_contract_confirmed)
        self.assertEqual((command.right_force_n,command.left_force_n),(1.,1.))
        self.n.time=10.1;self.n.params['software_estop']=False
        self.assertTrue(self.n.autonomy_callback(NS(data=True),NS()).success)
        self.n.force_callback(command);self.n.update()
        self.assertAlmostEqual(self.last().z,150.);self.assertAlmostEqual(self.last().y,50.)
