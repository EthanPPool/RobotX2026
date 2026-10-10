from setuptools import find_packages, setup

package_name = 'uav_safety'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    py_modules=['geofence_monitor'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RobotX UAV Team',
    maintainer_email='robotxuav@example.com',
    description='Latched flight-readiness and autonomy safety supervisor.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'safety_supervisor = uav_safety.safety_supervisor:main',
            'uav_geofence = geofence_monitor:main',
        ],
    },
)
