"""Receive UAV photographs on the USV Jetson with complete capture georeference."""
import json
import os
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from robotx_coordination.geometry import fresh, reference_id, rotation
from robotx_coordination.ros_support import now_sec, quaternion, seconds
from robotx_coordination_interfaces.msg import GeoImage


class SurveyRecorder(Node):
    def __init__(self):
        super().__init__('uav_survey_recorder')
        p=lambda n,d:self.declare_parameter(n,d).value
        self.origin=(p('origin_latitude',0.),p('origin_longitude',0.),p('water_altitude',0.))
        self.ref=reference_id(*self.origin);self.frame=p('map_frame','robotx_map')
        self.configured=p('datum_configured',False)
        self.directory=Path(p('output_directory','~/robotx_survey')).expanduser()
        self.max_images=int(p('max_images',1000));self.count=0;self.last_stamp=-1.
        if self.max_images<1:raise ValueError('max_images must be positive')
        self.create_subscription(GeoImage,'/uav/mapping/georeferenced_image',self.receive,2)

    def receive(self,msg):
        stamp=seconds(msg.header.stamp)
        if not self.configured or self.count>=self.max_images or stamp<=self.last_stamp or not fresh(stamp,now_sec(self),3.):
            return
        if msg.header.frame_id!=self.frame or msg.reference_id!=self.ref or len(msg.image.data)>5_000_000 or msg.image.format!='jpeg':
            return
        try:
            if reference_id(msg.origin_latitude,msg.origin_longitude,msg.water_altitude)!=self.ref:
                return
            rotation(quaternion(msg.camera_pose.orientation))
            xyz=msg.camera_pose.position
            if not all(np.isfinite(v) for v in (xyz.x,xyz.y,xyz.z)):
                return
            image=cv2.imdecode(np.frombuffer(bytes(msg.image.data),dtype=np.uint8),cv2.IMREAD_COLOR)
            if image is None or image.shape[:2]!=(msg.calibration.height,msg.calibration.width):
                return
            metadata={'reference_id':self.ref,'frame':self.frame,'stamp':stamp,'water_datum':list(self.origin),
                'camera_position':[xyz.x,xyz.y,xyz.z],'camera_quaternion_xyzw':list(quaternion(msg.camera_pose.orientation)),
                'camera_frame':msg.image.header.frame_id,'K':list(msg.calibration.k),'D':list(msg.calibration.d),
                'width':msg.calibration.width,'height':msg.calibration.height,'distortion_model':msg.calibration.distortion_model}
            self.directory.mkdir(parents=True,exist_ok=True)
            name=f'{msg.header.stamp.sec}_{msg.header.stamp.nanosec:09d}'
            photo=self.directory/(name+'.jpg');sidecar=self.directory/(name+'.json')
            photo.with_suffix('.jpg.tmp').write_bytes(bytes(msg.image.data))
            sidecar.with_suffix('.json.tmp').write_text(json.dumps(metadata,allow_nan=False,indent=2)+'\n')
            os.replace(photo.with_suffix('.jpg.tmp'),photo);os.replace(sidecar.with_suffix('.json.tmp'),sidecar)
            self.last_stamp=stamp;self.count+=1
        except (OSError,ValueError,cv2.error) as exc:
            self.get_logger().error('Unable to record georeferenced photo: '+str(exc))


def main(args=None):
    rclpy.init(args=args);node=SurveyRecorder()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
