import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'uav_bringup'
setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RobotX UAV Team',
    maintainer_email='robotxuav@example.com',
    description='RobotX UAV launch and configuration package.',
    license='Apache-2.0',
)
