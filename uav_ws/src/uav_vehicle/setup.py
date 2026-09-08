from setuptools import find_packages, setup

package_name = 'uav_vehicle'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RobotX UAV Team',
    maintainer_email='robotxuav@example.com',
    description='Safety-gated MAVROS vehicle and ArduCopter command interface.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
	    'por_route_runner = uav_vehicle.por_route_runner:main',
	    'por_element3_runner = uav_vehicle.por_element3_runner:main',
            'mavros_command_bridge = uav_vehicle.mavros_command_bridge:main',
            'mock_mavros = uav_vehicle.mock_mavros:main',
            'vehicle_manager = uav_vehicle.vehicle_manager:main',
        ],
    },
)
