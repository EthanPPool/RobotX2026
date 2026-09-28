from setuptools import find_packages, setup

package_name = "uuv_dashboard_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RobotX Team",
    maintainer_email="robotx@localhost",
    description="RobotX 2026 UUV package",
    license="MIT",
    entry_points={
        "console_scripts": [
            'bridge = uuv_dashboard_bridge.bridge:main',
        ],
    },
)
