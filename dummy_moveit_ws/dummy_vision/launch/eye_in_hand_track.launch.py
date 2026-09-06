#!/usr/bin/env python3
"""Eye-in-hand tracking bringup (vision + optional Servo stack).

Default dry_run:=true — publishes desired EE pose / TF only (safe for remote).
Set dry_run:=false at home after CDC bridge is ready.

Example:
  ros2 launch dummy_vision eye_in_hand_track.launch.py
  ros2 launch dummy_vision eye_in_hand_track.launch.py start_realsense:=false dry_run:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    start_realsense = LaunchConfiguration("start_realsense")
    start_servo = LaunchConfiguration("start_servo")
    dry_run = LaunchConfiguration("dry_run")
    use_easy_handeye = LaunchConfiguration("use_easy_handeye")
    follow_orientation = LaunchConfiguration("follow_orientation")
    marker_length = LaunchConfiguration("marker_length")

    actions = [
        DeclareLaunchArgument(
            "start_realsense",
            default_value="true",
            description="Launch realsense2_camera rs_launch",
        ),
        DeclareLaunchArgument(
            "start_servo",
            default_value="false",
            description="Also launch MoveIt servo_streaming (needs built workspace)",
        ),
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="If true, do not publish Servo twists",
        ),
        DeclareLaunchArgument(
            "use_easy_handeye",
            default_value="false",
            description="Use saved easy_handeye2 calib instead of static YAML",
        ),
        DeclareLaunchArgument(
            "follow_orientation",
            default_value="true",
            description="Track marker tilt as well as position",
        ),
        DeclareLaunchArgument("marker_length", default_value="0.05"),
    ]

    try:
        rs_share = get_package_share_directory("realsense2_camera")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(rs_share, "launch", "rs_launch.py")]
                ),
                launch_arguments={
                    "align_depth.enable": "true",
                    "enable_sync": "true",
                    # D415 on USB2 often rejects 1280x720; 640x480 is reliable.
                    "rgb_camera.color_profile": "640,480,15",
                }.items(),
                condition=IfCondition(start_realsense),
            )
        )
    except Exception:
        actions.append(
            LogInfo(msg="realsense2_camera not installed; set start_realsense:=false")
        )

    try:
        moveit_share = get_package_share_directory("dummy_moveit_config")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(moveit_share, "launch", "servo_streaming.launch.py")]
                ),
                condition=IfCondition(start_servo),
            )
        )
    except Exception:
        actions.append(
            LogInfo(msg="dummy_moveit_config not found; set start_servo:=false")
        )

    # When RealSense is up it owns camera_link->optical; otherwise allow optical stub.
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory("dummy_vision"),
                        "launch",
                        "eye_in_hand_publish.launch.py",
                    )
                ]
            ),
            launch_arguments={
                "use_easy_handeye": use_easy_handeye,
                "publish_optical_stub": "false",
            }.items(),
        )
    )

    actions.append(
        Node(
            package="dummy_vision",
            executable="aruco_detector_node",
            name="aruco_detector",
            output="screen",
            parameters=[
                {
                    "marker_length": ParameterValue(marker_length, value_type=float),
                    "marker_frame": "camera_marker",
                    "optical_frame": "camera_color_optical_frame",
                    "aruco_dict": "ORIGINAL",
                }
            ],
        )
    )

    actions.append(
        Node(
            package="dummy_vision",
            executable="aruco_servo_tracker_node",
            name="aruco_servo_tracker",
            output="screen",
            parameters=[
                {
                    "dry_run": ParameterValue(dry_run, value_type=bool),
                    "follow_orientation": ParameterValue(follow_orientation, value_type=bool),
                    "desired_marker_in_ee_x": 0.0,
                    "desired_marker_in_ee_y": 0.0,
                    "desired_marker_in_ee_z": 0.25,
                    "max_linear_vel": 0.08,
                    "max_angular_vel": 0.4,
                }
            ],
        )
    )

    return LaunchDescription(actions)
