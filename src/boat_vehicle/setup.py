from setuptools import find_packages, setup
import glob
import os

package_name = 'boat_vehicle'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/boat_vehicle']),
        ('share/boat_vehicle', ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob.glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='leo@todo.todo',
    description='Safety-gated MAVROS interface for RobotX BlueBoat.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'mavros_command_bridge = boat_vehicle.mavros_command_bridge:main',
		'esp32_status_bridge = boat_vehicle.esp32_status_bridge:main',
        ],
    },
)
