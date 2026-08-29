from setuptools import find_packages, setup
import glob
import os

package_name = 'boat_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/boat_dashboard'],
        ),
        (
            'share/boat_dashboard',
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob.glob(os.path.join('launch', '*.launch.py')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='leo@todo.todo',
    description='Browser test dashboard for RobotX BlueBoat.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'dashboard = boat_dashboard.dashboard:main',
        ],
    },
)
