from setuptools import find_packages, setup
from ros_robot_driver_wrapper.config import PROGRAM_VERSION

package_name = "ros_robot_driver_wrapper"

setup(
    name=package_name,
    version=PROGRAM_VERSION,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Samyar Sadat Akhavi",
    maintainer_email="samyarsadat@gigawhat.net",
    description="The ROS Robot Project: ROS 2 wrapper for robot hardware driver.",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
                "ros_robot_driver = ros_robot_driver_wrapper.main:main",
        ],
    },
)
