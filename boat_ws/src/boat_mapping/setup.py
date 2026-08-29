from setuptools import find_packages, setup
from glob import glob
import os

package_name = "boat_mapping"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="leo",
    maintainer_email="leo@example.com",
    description="Temporary RX26 placeholder map publisher and map assets.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "empty_map_publisher = boat_mapping.empty_map_publisher:main",
        ],
    },
)