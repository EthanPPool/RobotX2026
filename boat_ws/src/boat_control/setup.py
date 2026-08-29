from setuptools import find_packages, setup
import glob
import os

package_name = 'boat_control'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/boat_control']),
        ('share/boat_control', ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob.glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='leo@todo.todo',
    description='Target-to-velocity controller for RobotX BlueBoat.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'target_controller = boat_control.target_controller:main',
            'simple_gate_follower = boat_control.simple_gate_follower:main',
            'simple_gate_follower = boat_control.simple_gate_follower:main'
        ],
    },
)


