from glob import glob
from setuptools import find_packages, setup

package_name = "barxt_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ksh",
    maintainer_email="ksh@example.com",
    description="ROS2 Python node for validated Blue Robotics BarXT pressure/depth sensor data.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "barxt_node = barxt_ros2.barxt_node:main",
        ],
    },
)

