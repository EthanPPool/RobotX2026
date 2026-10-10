"""Capture georeferenced images and semantic buoy observations for the USV."""
import copy
import json
import time

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from robotx_coordination.geometry import reference_id, fresh
from robotx_coordination.mapping import SurveyMemory, project_image
from robotx_coordination.ros_support import now_sec, seconds, quaternion, point
from robotx_coordination_interfaces.msg import BuoyMap, GeoImage, SurveyBuoy, SurveyFootprint


class UavMapper(Node):
    def __init__(self):
        super().__init__('uav_mapper')
        p = lambda n,d: self.declare_parameter(n,d).value
        self.frame = p('map_frame','robotx_map')
        self.origin = (p('origin_latitude',0.),p('origin_longitude',0.),p('water_altitude',0.))
        self.ref = reference_id(*self.origin)
        self.uncertainty = float(p('georef_uncertainty',0.75))
        self.max_rate = float(p('max_rate',2.))
        self.max_age = float(p('image_timeout',0.5))
        self.max_range = float(p('max_mapping_range',100.))
        self.enabled = bool(p('datum_configured',False))
        if min(self.max_rate,self.max_age,self.max_range,self.uncertainty) <= 0:
            raise ValueError('Mapping limits must be positive')
        self.memory = SurveyMemory(ttl=float(p('survey_ttl',60.)))
        self.bridge = CvBridge(); self.info = None; self.last_run = -1e9
        self.pending_image = None
        self.buffer = Buffer(cache_time=Duration(seconds=5.))
        self.listener = TransformListener(self.buffer,self)
        self.map_pub = self.create_publisher(BuoyMap,'/uav/mapping/buoys',10)
        self.image_pub = self.create_publisher(GeoImage,'/uav/mapping/georeferenced_image',2)
        self.status_pub = self.create_publisher(String,'/uav/mapping/status',10)
        self.create_subscription(CameraInfo,p('camera_info_topic','/uav/downward/camera_info'),self.on_info,qos_profile_sensor_data)
        self.create_subscription(Image,p('image_topic','/uav/downward/image_raw'),self.on_image,qos_profile_sensor_data)
        self.create_timer(0.05,self.process_pending)

    def on_info(self,msg):
        self.info = msg

    def status(self,reason):
        msg = String(); msg.data = json.dumps({'mapping_state':reason,'reference_id':self.ref})
        self.status_pub.publish(msg)

    def on_image(self,msg):
        self.pending_image = msg

    def process_pending(self):
        if self.pending_image is None:
            return
        self.process_image(self.pending_image)

    def process_image(self,msg):
        if time.monotonic()-self.last_run < 1/self.max_rate:
            return
        if not self.enabled:
            self.status('DATUM_NOT_CONFIGURED'); return
        stamp, now = seconds(msg.header.stamp), now_sec(self)
        if not fresh(stamp,now,self.max_age):
            self.status('IMAGE_STALE'); return
        if self.info is None or self.info.header.frame_id != msg.header.frame_id:
            self.status('CALIBRATION_FRAME_MISMATCH'); return
        if self.info.width != msg.width or self.info.height != msg.height or self.info.distortion_model not in ('','plumb_bob','rational_polynomial'):
            self.status('CALIBRATION_UNSUPPORTED'); return
        try:
            # Exact acquisition time; never use the latest pose for an old image.
            tf = self.buffer.lookup_transform(self.frame,msg.header.frame_id,Time.from_msg(msg.header.stamp))
            t = tf.transform.translation
            pose = Pose(); pose.position = point((t.x,t.y,t.z)); pose.orientation = tf.transform.rotation
            image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            observations, footprint = project_image(image,self.info.k,self.info.d,
                (t.x,t.y,t.z),quaternion(pose.orientation),stamp,self.uncertainty,self.max_range)
            if not self.memory.add(observations,footprint,now):
                self.status('REPLAY_OR_STALE'); return
            ok, jpeg = cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,80])
            if not ok:
                raise ValueError('JPEG encoding failed')
        except TransformException:
            # TF and image DDS delivery can arrive in either order. Retry only
            # this latest image until its original acquisition stamp expires.
            self.status('WAITING_FOR_CAPTURE_TF'); return
        except (ValueError,cv2.error,CvBridgeError) as exc:
            self.status('REJECTED: '+str(exc)); return
        self.pending_image = None
        self.last_run = time.monotonic()
        mapped = BuoyMap(); mapped.header = copy.deepcopy(msg.header); mapped.header.frame_id = self.frame
        mapped.reference_id = self.ref
        mapped.origin_latitude,mapped.origin_longitude,mapped.water_altitude = self.origin
        for o in self.memory.obstacles:
            b = SurveyBuoy(); b.position = point((o.x,o.y,0.)); b.radius = o.radius
            b.uncertainty = o.uncertainty; b.color = o.color
            b.observed_at = Time(seconds=o.stamp).to_msg(); mapped.buoys.append(b)
        for f in self.memory.footprints:
            out = SurveyFootprint(); out.observed_at = Time(seconds=f.stamp).to_msg()
            out.corners = [point((x,y,0.)) for x,y in f.points]; mapped.footprints.append(out)
        self.map_pub.publish(mapped)
        geo = GeoImage(); geo.header = mapped.header; geo.reference_id = self.ref
        geo.origin_latitude,geo.origin_longitude,geo.water_altitude = self.origin
        geo.camera_pose = pose; geo.calibration = self.info
        geo.image = CompressedImage(); geo.image.header = msg.header
        geo.image.format = 'jpeg'; geo.image.data = jpeg.tobytes()
        self.image_pub.publish(geo); self.status('MAPPING')


def main(args=None):
    rclpy.init(args=args); node = UavMapper()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
