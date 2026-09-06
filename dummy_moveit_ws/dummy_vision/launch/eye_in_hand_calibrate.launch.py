#!/usr/bin/env python3
"""Eye-in-hand calibration bringup (run at home with arm + camera).

Prerequisites (install once):
  sudo apt install ros-${ROS_DISTRO}-easy-handeye2 ros-${ROS_DISTRO}-realsense2-camera

Move the arm to varied poses with the marker fixed in view, Take Sample, then Save.
Result: ~/.ros2/easy_handeye2/calibrations/dummy_eih_calib.calib
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup_calibrate(context, *args, **kwargs):
    actions = []
    try:
        easy_share = get_package_share_directory("easy_handeye2")
    except Exception as exc:
        return [
            LogInfo(
                msg=(
                    f"easy_handeye2 not installed ({exc}). "
                    "Install: sudo apt install ros-$ROS_DISTRO-easy-handeye2"
                )
            )
        ]

    calib_name = LaunchConfiguration("calibration_name").perform(context)
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(easy_share, "launch", "calibrate.launch.py")]
            ),
            launch_arguments={
                "name": calib_name,
                "calibration_type": "eye_in_hand",
                "robot_base_frame": "base_link",
                "robot_effector_frame": "link6_1_1",
                "tracking_base_frame": "camera_link",
                "tracking_marker_frame": "camera_marker",
            }.items(),
        )
    )
    actions.append(
        LogInfo(
            msg=(
                f"easy_handeye2 calibrate ({calib_name}): take samples then Save. "
                "Need MoveIt / robot TF running in another terminal."
            )
        )
    )
    return actions


def generate_launch_description():
    start_realsense = LaunchConfiguration("start_realsense")
    marker_length = LaunchConfiguration("marker_length")
    marker_id = LaunchConfiguration("marker_id")

    realsense_actions = []
    try:
        rs_share = get_package_share_directory("realsense2_camera")
        realsense_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(rs_share, "launch", "rs_launch.py")]
                ),
                launch_arguments={
                    "align_depth.enable": "true",
                    "enable_sync": "true",
                    # D415 on USB2 often rejects 1280x720
                    "rgb_camera.color_profile": "640,480,15",
                }.items(),
                condition=IfCondition(start_realsense),
            )
        )
    except Exception:
        realsense_actions.append(
            LogInfo(
                msg="realsense2_camera not found; start camera yourself or install the package"
            )
        )

    detector = Node(
        package="dummy_vision",
        executable="aruco_detector_node",
        name="aruco_detector",
        output="screen",
        parameters=[
            {
                "marker_length": ParameterValue(
                    LaunchConfiguration("marker_length"), value_type=float
                ),
                "marker_id": ParameterValue(
                    LaunchConfiguration("marker_id"), value_type=int
                ),
                "marker_frame": "camera_marker",
                "optical_frame": "camera_color_optical_frame",
                "aruco_dict": "ORIGINAL",
            }
        ],
    )

    # RealSense may not publish optical TF when camera_link is already claimed by
    # easy_handeye2's dummy ee->camera publisher. Provide optical stub always for calib.
    optical_stub = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_optical_stub",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "-1.57079632679",
            "--pitch",
            "0",
            "--yaw",
            "-1.57079632679",
            "--frame-id",
            "camera_link",
            "--child-frame-id",
            "camera_color_optical_frame",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_realsense", default_value="true"),
            DeclareLaunchArgument("marker_length", default_value="0.05"),
            DeclareLaunchArgument("marker_id", default_value="0"),
            DeclareLaunchArgument("calibration_name", default_value="dummy_eih_calib"),
            *realsense_actions,
            optical_stub,
            detector,
            OpaqueFunction(function=_setup_calibrate),
        ]
    )
