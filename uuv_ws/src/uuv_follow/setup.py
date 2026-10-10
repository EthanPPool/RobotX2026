from setuptools import find_packages, setup
from glob import glob
name = 'uuv_follow'
setup(name=name, version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages',['resource/'+name]),
                ('share/'+name,['package.xml']),
                ('share/'+name+'/launch',glob('launch/*.launch.py')),
                ('share/'+name+'/config',glob('config/*.yaml'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='RobotX Team', maintainer_email='robotx@localhost',
    description='Shared-datum RobotX UAV survey and UUV following', license='MIT',
    entry_points={'console_scripts': ['follower = uuv_follow.follower:main', 'velocity_guard = uuv_follow.velocity_guard:main']})
