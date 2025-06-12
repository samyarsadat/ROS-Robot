import os
from setuptools import find_packages, setup
from glob import glob

package_name = "ros_robot_bringup"
package_version = "2025.1.22"

setup(
    name=package_name,
    version=package_version,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*_launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Samyar Sadat Akhavi",
    maintainer_email="samyarsadat@gigawhat.net",
    description="The ROS Robot Project: Robot infrastructure bringup package.",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={},
)