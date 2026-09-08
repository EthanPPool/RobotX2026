from setuptools import find_packages, setup

package_name = 'uav_perception'
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
    description='Normalized target interface for future camera/perception nodes.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'target_filter = uav_perception.target_filter:main',
        'mock_target_publisher = uav_perception.mock_target_publisher:main',
    ]},
)
