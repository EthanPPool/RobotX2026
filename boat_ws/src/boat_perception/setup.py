from setuptools import find_packages, setup
import glob
import os

package_name = 'boat_perception'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/boat_perception']),
        ('share/boat_perception', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob.glob(os.path.join('config', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='leo@todo.todo',
    description='LiDAR buoy and gate perception for RobotX.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'buoy_detector = boat_perception.buoy_detector:main',
            'gate_detector = boat_perception.gate_detector:main'
        ],
    },
)
