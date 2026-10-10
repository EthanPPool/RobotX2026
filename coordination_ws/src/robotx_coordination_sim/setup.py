from glob import glob
from setuptools import setup, find_packages
setup(name='robotx_coordination_sim',version='0.1.0',packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/robotx_coordination_sim']),
 ('share/robotx_coordination_sim',['package.xml']),('share/robotx_coordination_sim/launch',glob('launch/*.launch.py'))],
 install_requires=['setuptools'],maintainer='RobotX Team',maintainer_email='robotx@localhost',
 description='Isolated software simulation of UAV survey, USV route, UUV trail',license='MIT',
 entry_points={'console_scripts':['sim_world = robotx_coordination_sim.sim_world:main']})
