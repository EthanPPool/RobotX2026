from glob import glob
from setuptools import find_packages, setup

name = 'robotx_buoy_interpretation'
setup(name=name, version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]),
                ('share/'+name, ['package.xml']),
                ('share/'+name+'/config', glob('config/*.yaml')),
                ('share/'+name+'/launch', glob('launch/*.launch.py'))],
    install_requires=['setuptools', 'numpy'], zip_safe=True,
    maintainer='RobotX Team', maintainer_email='robotx@example.com', license='MIT',
    description='Shared buoy interpretation for USV-only and UAV-assisted operation',
    entry_points={'console_scripts': [
        'usv_buoy_interpretation = robotx_buoy_interpretation.usv_node:main',
        'uav_buoy_interpretation = robotx_buoy_interpretation.uav_node:main',
        'buoy_tracker = robotx_buoy_interpretation.tracker_node:main',
    ]})
