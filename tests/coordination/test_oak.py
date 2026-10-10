"""Second-device binding, capture timing and calibrated output geometry."""
import importlib.util
import math
from datetime import timedelta
from pathlib import Path
from types import SimpleNamespace as Box

import pytest

from boat_stack_shim import Node, Publisher, load_class, load_module

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('oak_support',
    ROOT / 'uav_ws/src/uav_mapping/uav_mapping/oak_support.py')
support = importlib.util.module_from_spec(spec)
spec.loader.exec_module(support)


def test_downward_device_id_is_required():
    with pytest.raises(ValueError, match='DeviceID'):
        support.require_device_id('  ')
    assert support.require_device_id(' second-mxid ') == 'second-mxid'


def test_acquisition_stamp_preserves_camera_latency():
    epoch = 1_791_700_000_000_000_000
    assert support.acquisition_nanoseconds(epoch, timedelta(seconds=40), timedelta(seconds=39.875), .5) == epoch - 125_000_000


@pytest.mark.parametrize('age', [.501, -0.002])
def test_stale_or_future_camera_frame_rejected(age):
    with pytest.raises(ValueError):
        support.acquisition_nanoseconds(10**18, timedelta(seconds=20), timedelta(seconds=20-age), .5)


def calibration(k=None, d=None, model='Perspective'):
    return Box(getIntrinsicMatrix=lambda: k if k is not None else [[400.,0.,320.],[0.,400.,240.],[0.,0.,1.]],
               getDistortionCoefficients=lambda: d if d is not None else [0.]*8,
               getDistortionModel=lambda: Box(name=model))


@pytest.mark.parametrize('bad', ['missing', 'fisheye', 'zero_k', 'nonfinite', 'bad_d'])
def test_unusable_oak_calibration_rejected(bad):
    transform = calibration()
    if bad == 'missing': transform = None
    elif bad == 'fisheye': transform = calibration(model='Fisheye')
    elif bad == 'zero_k': transform = calibration(k=[[0.]*3]*3)
    elif bad == 'nonfinite': transform = calibration(d=[math.nan]*8)
    else: transform = calibration(d=[0.]*6)
    with pytest.raises(ValueError): support.frame_calibration(transform)


def test_output_calibration_keeps_rational_distortion():
    k, d, model = support.frame_calibration(calibration(d=[.1,.2,.3,.4,.5,.6,.7,.8]))
    assert k[2] == 320. and d == [.1,.2,.3,.4,.5,.6,.7,.8] and model == 'rational_polynomial'


def test_real_depthai_frame_transform_scales_intrinsics():
    dai = pytest.importorskip('depthai')
    transform = dai.ImgTransformation(1280, 960, [[800.,0.,640.],[0.,800.,480.],[0.,0.,1.]],
                                      dai.CameraModel.Perspective, [0.]*8)
    transform.addScale(.5, .5)
    k, d, model = support.frame_calibration(transform)
    assert k == pytest.approx([400.,0.,320.,0.,400.,240.,0.,0.,1.])


def test_dual_launch_refuses_same_or_missing_device():
    for primary, downward in [('same','same'), ('','second'), ('first','')]:
        class Configuration:
            def __init__(self, key): self.key = key
            def perform(self, context): return context[self.key]
        module = load_module(ROOT/'uav_ws/src/uav_bringup/launch/dual_oak_mapping.launch.py',
                             {'LaunchConfiguration': Configuration})
        with pytest.raises(ValueError, match='DIFFERENT'):
            module['camera_nodes']({'primary_device_id': primary, 'downward_device_id': downward})


def test_downward_node_binds_second_device_and_publishes_matching_capture_headers():
    import numpy as np
    opened = []
    device = Box(getDeviceId=lambda: 'second', close=lambda: None)
    frame = Box(getWidth=lambda: 640, getHeight=lambda: 480, getTransformation=lambda: calibration(),
                getTimestamp=lambda: timedelta(seconds=9.9), getCvFrame=lambda: np.zeros((480,640,3), np.uint8))
    queue = Box(tryGet=lambda: frame)
    output = Box(createOutputQueue=lambda **kwargs: queue)
    camera = Box(requestOutput=lambda **kwargs: output)
    builder = Box(build=lambda *args: camera)
    pipeline = Box(create=lambda *args: builder, start=lambda: None, stop=lambda: None)
    fake_dai = Box(DeviceInfo=lambda device_id: device_id,
        Device=lambda info: opened.append(info) or device,
        Pipeline=lambda dev: pipeline, node=Box(Camera=Box), CameraBoardSocket=Box(CAM_A='CAM_A'),
        ImgResizeMode=Box(STRETCH='stretch'), ImgFrame=Box(Type=Box(BGR888p='bgr')),
        Clock=Box(now=lambda: timedelta(seconds=10.)))
    class CameraNode(Node):
        def declare_parameter(self, name, default):
            return super().declare_parameter(name, 'second' if name == 'camera_device_id' else default)
        def get_parameter(self, name):
            return Box(value=False) if name == 'use_sim_time' else super().get_parameter(name)
    class RosTime:
        def __init__(self, nanoseconds, clock_type): self.ns = nanoseconds
        def to_msg(self): return Box(sec=self.ns//10**9, nanosec=self.ns%10**9)
    class Clock:
        def now(self): return Box(nanoseconds=10**10, clock_type='ROS')
    info = lambda: Box(header=Box())
    context = {'Node': CameraNode, 'dai': fake_dai, 'Time': RosTime, 'CameraInfo': info, 'Image': Box,
               'CvBridge': lambda: Box(cv2_to_imgmsg=lambda *a,**kw: Box(header=Box())),
               'require_device_id': support.require_device_id, 'frame_calibration': support.frame_calibration,
               'acquisition_nanoseconds': support.acquisition_nanoseconds}
    cls = load_class(ROOT/'uav_ws/src/uav_mapping/uav_mapping/downward_camera.py', 'DownwardCamera', context)
    node = cls(); node.get_clock = lambda: Clock(); node.tick()
    assert opened == ['second']
    assert node.image_pub.last.header == node.info_pub.last.header
    assert node.image_pub.last.header.stamp.sec == 9
    assert node.image_pub.last.header.stamp.nanosec == 900_000_000
    assert node.info_pub.last.k[0] == 400.
    frame.getTimestamp = lambda: timedelta(seconds=9.)
    node.image_pub.last = None; node.tick()
    assert node.image_pub.last is None
