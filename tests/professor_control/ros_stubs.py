"""ROS message/transport stubs. These tests do not validate DDS or generated code."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType,SimpleNamespace as NS
from unittest.mock import patch


def stamp(t):return NS(sec=int(t),nanosec=round((t-int(t))*1e9))
def header(t=10.,frame='robotx_map'):return NS(stamp=stamp(t),frame_id=frame)


class Now:
    def __init__(self,t):self.nanoseconds=round(t*1e9)
    def to_msg(self):return stamp(self.nanoseconds/1e9)
    def __sub__(self,other):return NS(nanoseconds=self.nanoseconds-other.nanoseconds)


class Message:
    def __init__(self):self.header=header()


class Twist(Message):
    def __init__(self):
        super().__init__();self.twist=NS(linear=NS(x=0.,y=0.,z=0.),angular=NS(x=0.,y=0.,z=0.))


class Parameter:
    Type=NS(BOOL=1)
    def __init__(self,name,kind=None,value=None):self.name=name;self.value=value


class FakeNode:
    overrides={}
    def __init__(self,name):
        self.params={};self.time=10.;self.pub={};self.sub={};self.name=name
    def declare_parameter(self,n,d):
        self.params[n]=self.overrides.get(n,d);return NS(value=self.params[n])
    def get_parameter(self,n):return NS(value=self.params[n])
    def set_parameters(self,params):
        for p in params:self.params[p.name]=p.value
    def get_clock(self):return NS(now=lambda:Now(self.time))
    def get_logger(self):return NS(**{k:lambda *a,**kw:None for k in ('info','warn','warning','error','debug')})
    def create_publisher(self,kind,topic,qos):
        values=[];self.pub[topic]=values;return NS(publish=values.append)
    def create_subscription(self,kind,topic,callback,qos):self.sub[topic]=callback;return object()
    def create_service(self,*args):return object()
    def create_timer(self,*args):return object()
    def create_client(self,*args):return NS(wait_for_service=lambda **kw:True,service_is_ready=lambda:True,
        call_async=lambda req:NS(done=lambda:False,add_done_callback=lambda f:None))


def load_nodes(root):
    definitions={'rclpy':{},'rclpy.node':{'Node':FakeNode},'rclpy.parameter':{'Parameter':Parameter},
        'rclpy.qos':{'qos_profile_sensor_data':object()},'geometry_msgs':{},'geometry_msgs.msg':{'TwistStamped':Twist},
        'nav_msgs':{},'nav_msgs.msg':{'Odometry':Message},'sensor_msgs':{},'sensor_msgs.msg':{'BatteryState':Message},
        'std_msgs':{},'std_msgs.msg':{'String':Message,'Bool':Message},'std_srvs':{},
        'std_srvs.srv':{'SetBool':object,'Trigger':object},'mavros_msgs':{},
        'mavros_msgs.msg':{'ManualControl':Message,'State':Message},'mavros_msgs.srv':{'SetMode':type('Mode',(),{'Request':Message})},
        'robotx_control_interfaces':{},'robotx_control_interfaces.msg':{'ThrusterForces':Message},
        'robotx_coordination_interfaces':{},'robotx_coordination_interfaces.msg':{'TrajectoryPreview':Message}}
    mods={}
    for name,values in definitions.items():
        m=ModuleType(name);m.__dict__.update(values);mods[name]=m
    def load(name,path):
        spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m);return m
    with patch.dict(sys.modules,mods):
        base=load('boat_vehicle.mavros_command_bridge',root/'boat_ws/src/boat_vehicle/boat_vehicle/mavros_command_bridge.py')
        sysmods={'boat_vehicle':ModuleType('boat_vehicle'),'boat_vehicle.mavros_command_bridge':base}
        node=load('robotx_professor_control._node_tests',root/'coordination_ws/src/robotx_professor_control/robotx_professor_control/node.py')
        sysmods['robotx_professor_control.node']=node
        with patch.dict(sys.modules,sysmods):
            bridge=load('robotx_professor_control._bridge_tests',root/'coordination_ws/src/robotx_professor_control/robotx_professor_control/bridge.py')
    return node,bridge,base,mods
