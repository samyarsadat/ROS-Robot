from setuptools.glob import glob
from setuptools import find_packages, setup
import os
package_name = "ros_robot_lidar"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Samyar Sadat Akhavi",
    maintainer_email="samyarsadat@gigawhat.net",
    description="LiDAR sensor-related configuration and launch files for The ROS Robot Project.",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={},
)