from setuptools import find_packages, setup
import glob
import os

package_name = 'boat_mission'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/boat_mission']),
        ('share/boat_mission', ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob.glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='leo@todo.todo',
    description='RobotX mission state machines.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'por_gate_mission = boat_mission.por_gate_mission:main'
        ],
    },
)
