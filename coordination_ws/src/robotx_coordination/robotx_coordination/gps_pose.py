"""GPS+ENU body-attitude adapter into the explicit shared water datum.

UAV altitude uses a calibrated downward range sensor, not relative takeoff height.
Underwater localization must be supplied by another estimator; this adapter is
for surface/air vehicles only. Sensor lever arms should be calibrated upstream.
"""
import math

import message_filters
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, Range
from tf2_ros import TransformBroadcaster

from .geometry import finite, fresh, geodetic_to_enu, reference_id, rotation
from .ros_support import now_sec, quaternion, seconds


class GpsPose(Node):
    def __init__(self):
        super().__init__('gps_shared_pose')
        p = lambda n,d:self.declare_parameter(n,d).value
        self.origin = (p('origin_latitude',0.),p('origin_longitude',0.),p('water_altitude',0.))
        reference_id(*self.origin)
        self.configured = p('datum_configured',False)
        self.vehicle = p('vehicle','uav')
        if self.vehicle not in ('uav','boat'):
            raise ValueError('GPS pose supports UAV or boat; UUV needs underwater localization')
        self.frame = p('map_frame','robotx_map'); self.body = p('body_frame',self.vehicle+'/base_link')
        self.max_variance = float(p('max_gps_variance',1.))
        self.pub = self.create_publisher(Odometry,p('output_topic','/'+self.vehicle+'/coordination/odometry'),10)
        self.broadcaster = TransformBroadcaster(self)
        prefix = '/mavros' if self.vehicle == 'boat' else '/uav/mavros'
        inputs = [message_filters.Subscriber(self,NavSatFix,p('gps_topic',prefix+'/global_position/global'),qos_profile=qos_profile_sensor_data),
                  message_filters.Subscriber(self,Imu,p('imu_topic',prefix+'/imu/data'),qos_profile=qos_profile_sensor_data)]
        if self.vehicle == 'uav':
            inputs.append(message_filters.Subscriber(self,Range,p('range_topic','/uav/downward/range'),qos_profile=qos_profile_sensor_data))
        self.sync = message_filters.ApproximateTimeSynchronizer(inputs,30,0.08)
        self.sync.registerCallback(self.on_sensors)

    def on_sensors(self,gps,imu,range_msg=None):
        if not self.configured:
            return
        now = now_sec(self)
        sensors = [gps,imu]+([range_msg] if range_msg is not None else [])
        if not all(fresh(seconds(m.header.stamp),now,0.3) for m in sensors):
            return
        c = gps.position_covariance
        if gps.status.status < 0 or gps.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            return
        if not finite(gps.latitude,gps.longitude,gps.altitude,c[0],c[4]) or not 0 <= c[0] <= self.max_variance or not 0 <= c[4] <= self.max_variance:
            return
        if imu.orientation_covariance[0] < 0:
            return
        try:
            rot = rotation(quaternion(imu.orientation))
            xyz = geodetic_to_enu(gps.latitude,gps.longitude,gps.altitude,self.origin)
        except ValueError:
            return
        xyz[2] = 0.  # Boat's water-plane reference, not GPS altitude noise.
        if range_msg is not None:
            value = range_msg.range
            if not finite(value) or not range_msg.min_range < value < range_msg.max_range or rot[2,2] < 0.9:
                return
            xyz[2] = value*rot[2,2]
        msg = Odometry(); msg.header.stamp = imu.header.stamp; msg.header.frame_id = self.frame
        msg.child_frame_id = self.body
        msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z = map(float,xyz)
        msg.pose.pose.orientation = imu.orientation
        msg.pose.covariance[0] = c[0]; msg.pose.covariance[7] = c[4]; msg.pose.covariance[14] = 0.04
        self.pub.publish(msg)
        tf = TransformStamped(); tf.header = msg.header; tf.child_frame_id = self.body
        tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z = map(float,xyz)
        tf.transform.rotation = imu.orientation; self.broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args); node = GpsPose()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
