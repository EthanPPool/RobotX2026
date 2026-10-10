"""Second OAK-D RGB camera, selected explicitly and stamped at acquisition."""
import copy

import depthai as dai
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from .oak_support import acquisition_nanoseconds, frame_calibration, require_device_id


class DownwardCamera(Node):
    def __init__(self):
        super().__init__('uav_downward_camera')
        p = lambda name, default: self.declare_parameter(name, default).value
        device_id = require_device_id(p('camera_device_id', ''))
        self.frame = p('camera_frame', 'uav/downward_optical')
        self.width = int(p('width', 640))
        self.height = int(p('height', 480))
        self.fps = float(p('fps', 10.0))
        self.max_age = float(p('image_timeout', 0.5))
        if self.width < 20 or self.height < 20 or not 0 < self.fps <= 60 or self.max_age <= 0:
            raise ValueError('Invalid OAK-D dimensions, rate or image timeout')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Hardware OAK-D driver requires a system ROS clock')
        self.device = dai.Device(dai.DeviceInfo(device_id))
        self.pipeline = None
        try:
            self.pipeline = dai.Pipeline(self.device)
            camera = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            # Explicit full-FOV stretch; frame metadata carries the actual output K.
            output = camera.requestOutput(
                size=(self.width, self.height), type=dai.ImgFrame.Type.BGR888p,
                resizeMode=dai.ImgResizeMode.STRETCH, fps=self.fps,
                enableUndistortion=False)
            self.queue = output.createOutputQueue(maxSize=1, blocking=False)
            self.pipeline.start()
        except Exception:
            self.device.close()
            raise
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/uav/downward/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/uav/downward/camera_info', qos_profile_sensor_data)
        self.get_logger().info(f'Downward OAK-D {self.device.getDeviceId()}; RGB CAM_A, {self.width}x{self.height}')
        self.timer = self.create_timer(1.0 / self.fps, self.tick)

    def tick(self):
        frame = self.queue.tryGet()
        if frame is None:
            return
        try:
            if (frame.getWidth(), frame.getHeight()) != (self.width, self.height):
                raise ValueError('OAK-D output size differs from configured size')
            k, d, model = frame_calibration(frame.getTransformation())
            now = self.get_clock().now()
            ns = acquisition_nanoseconds(now.nanoseconds, dai.Clock.now(), frame.getTimestamp(), self.max_age)
            stamp = Time(nanoseconds=ns, clock_type=now.clock_type).to_msg()
            image = frame.getCvFrame()
            if image.shape[:2] != (self.height, self.width):
                raise ValueError('Decoded image size differs from calibration')
        except (ValueError, RuntimeError) as error:
            self.get_logger().error(str(error), throttle_duration_sec=2.0)
            return
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame
        info.width, info.height = self.width, self.height
        info.k, info.d, info.distortion_model = k, d, model
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, k[6], k[7], k[8], 0.0]
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header = copy.deepcopy(info.header)
        self.info_pub.publish(info)
        self.image_pub.publish(msg)

    def destroy_node(self):
        try:
            if self.pipeline is not None:
                self.pipeline.stop()
        finally:
            self.device.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DownwardCamera()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
