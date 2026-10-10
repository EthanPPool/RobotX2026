"""Execute the repository's actual controller/bridge on a tiny ROS API shim.

This validates Python behavior and completion handling, not DDS or autopilot
physics. No copied steering law is used. A real ROS launch is also provided.
"""
import ast
import copy
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace as Box


class Stamp:
    def __init__(self,seconds=0.):
        self.nanoseconds=round(seconds*1e9)

    def __sub__(self,other):return Box(nanoseconds=self.nanoseconds-other.nanoseconds)
    def to_msg(self):return Box(sec=self.nanoseconds//10**9,nanosec=self.nanoseconds%10**9)


class TwistStamped:
    def __init__(self):
        self.header=Box(stamp=Stamp().to_msg(),frame_id='')
        self.twist=Box(linear=Box(x=0.,y=0.,z=0.),angular=Box(x=0.,y=0.,z=0.))


class NavigationTarget:
    def __init__(self):
        self.header=Box(stamp=Stamp().to_msg(),frame_id='base_link')
        self.target=Box(x=0.,y=0.,z=0.);self.desired_speed=0.;self.stop=True


class ValueMsg:
    def __init__(self):self.data=None


class Parameter:
    Type=Box(BOOL=bool)
    def __init__(self,name,kind=None,value=None):self.name=name;self.value=value


class Publisher:
    def __init__(self):self.last=None
    def publish(self,msg):self.last=copy.deepcopy(msg)


class Future:
    def done(self):return True
    def result(self):return Box(mode_sent=True,success=True)
    def add_done_callback(self,callback):callback(self)


class Node:
    clock=0.
    def __init__(self,name):self.params={};self._mode_hook=lambda mode:None
    def declare_parameter(self,name,default):
        self.params[name]=False if name=='battery_required_for_propulsion' else default
        return Box(value=self.params[name])
    def get_parameter(self,name):return Box(value=self.params[name])
    def set_parameters(self,values):
        for p in values:self.params[p.name]=p.value
        return []
    def get_clock(self):return Box(now=lambda:Stamp(Node.clock))
    def get_logger(self):return Box(info=lambda *a,**k:None,warn=lambda *a,**k:None,warning=lambda *a,**k:None,error=lambda *a,**k:None)
    def create_publisher(self,*a,**k):return Publisher()
    def create_subscription(self,*a,**k):return None
    def create_timer(self,*a,**k):return None
    def create_service(self,*a,**k):return None
    def create_client(self,*a,**k):
        def call(req):self._mode_hook(req.custom_mode);return Future()
        return Box(service_is_ready=lambda:True,call_async=call)


def load_module(path,extra=None):
    tree=ast.parse(Path(path).read_text())
    tree.body=[n for n in tree.body if not isinstance(n,(ast.Import,ast.ImportFrom))]
    context={'__name__':'shim_loaded','Node':Node,'Parameter':Parameter,'TwistStamped':TwistStamped,
        'NavigationTarget':NavigationTarget,'State':Box,'ManualControl':TwistStamped,
        'Bool':ValueMsg,'String':ValueMsg,'BatteryState':Box,'SetBool':Box,'Trigger':Box,
        'SetMode':Box(Request=Box),'qos_profile_sensor_data':None,
        'copy':copy,'json':json,'math':math,'time':time}
    context.update(extra or {})
    exec(compile(tree,str(path),'exec'),context)
    return context


def load_class(path,name,extra=None):
    return load_module(path,extra)[name]


class ActualBoatStack:
    def __init__(self,repo):
        self.controller=load_class(Path(repo)/'boat_ws/src/boat_control/boat_control/target_controller.py','TargetController')()
        self.bridge=load_class(Path(repo)/'boat_ws/src/boat_vehicle/boat_vehicle/mavros_command_bridge.py','MavrosCommandBridge')()
        self.mode='GUIDED';self.state=Box(connected=True,armed=True,mode=self.mode)
        self.bridge._mode_hook=self.set_mode
        self.bridge.state_callback(self.state)
        self.bridge.estop_callback(Box(data=False),Box())
        response=self.bridge.autonomy_callback(Box(data=True),Box())
        if not response.success:raise RuntimeError(response.message)

    def set_mode(self,mode):
        self.mode=mode;self.state.mode=mode;self.bridge.state_callback(self.state)

    def step(self,t,target):
        Node.clock=t
        x,y,speed,reason=target
        msg=NavigationTarget();msg.target.x=x;msg.target.y=y;msg.desired_speed=speed;msg.stop=reason!='ROUTE_TRACKING'
        self.controller.target_callback(msg);self.controller.update()
        self.bridge.command_callback(self.controller.cmd_pub.last)
        state=ValueMsg();state.data=reason;self.bridge.mission_state_callback(state)
        self.bridge.update()
        out=self.bridge.cmd_pub.last
        if out is None:return 0.,0.
        return float(out.twist.linear.x),float(out.twist.angular.z)
