from glob import glob
from setuptools import find_packages, setup

name = 'robotx_competition_planner'
setup(name=name, version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]),
                ('share/'+name, ['package.xml']),
                ('share/'+name+'/config', glob('config/*.yaml')),
                ('share/'+name+'/launch', glob('launch/*.launch.py'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='RobotX Team', maintainer_email='robotx@example.com', license='MIT',
    description='Competition Safe Passage routing and trajectory references',
    entry_points={'console_scripts': [
        'competition_planner = robotx_competition_planner.node:main']})
