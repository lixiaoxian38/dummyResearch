import os
from glob import glob

from setuptools import find_packages, setup

package_name = "dummy_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lxx",
    maintainer_email="lxx@todo.todo",
    description="Eye-in-hand ArUco detection and servo tracking for Dummy arm",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aruco_detector_node = dummy_vision.aruco_detector_node:main",
            "aruco_tracker_node = dummy_vision.aruco_tracker_node:main",
            "aruco_servo_tracker_node = dummy_vision.aruco_servo_tracker_node:main",
            "depth_detector_node = dummy_vision.depth_detector_node:main",
        ],
    },
)
