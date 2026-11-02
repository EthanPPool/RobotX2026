from glob import glob
from setuptools import find_packages,setup
name='robotx_professor_control'
setup(name=name,version='0.1.0',packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages',['resource/'+name]),
                  ('share/'+name,['package.xml']),('share/'+name+'/config',glob('config/*')),
                  ('share/'+name+'/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools'],zip_safe=True,maintainer='RobotX Team',
      maintainer_email='robotx@example.com',description='Two-thruster professor control law',license='MIT',
      entry_points={'console_scripts':['professor_controller = robotx_professor_control.node:main',
                                      'professor_force_bridge = robotx_professor_control.bridge:main']})
