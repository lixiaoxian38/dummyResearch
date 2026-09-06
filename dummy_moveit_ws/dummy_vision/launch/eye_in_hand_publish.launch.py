#!/usr/bin/env python3
"""Publish eye-in-hand extrinsics.

Default: static placeholder from config/handeye_static.yaml (dry-run / remote).
Optional: use_easy_handeye:=true to launch easy_handeye2 publish for dummy_eih_calib
(requires ros-${ROS_DISTRO}-easy-handeye2 installed).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    use_easy = LaunchConfiguration("use_easy_handeye").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )
    actions = []

    if use_easy:
        calib_name = LaunchConfiguration("calibration_name").perform(context)
        try:
            easy_share = get_package_share_directory("easy_handeye2")
        except Exception as exc:
            return [
                LogInfo(
                    msg=f"easy_handeye2 not found ({exc}); install it or use_easy_handeye:=false"
                )
            ]
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(easy_share, "launch", "publish.launch.py")]
                ),
                launch_arguments={"name": calib_name}.items(),
            )
        )
        actions.append(LogInfo(msg=f"Publishing easy_handeye2 calibration: {calib_name}"))
        return actions

    cfg_arg = LaunchConfiguration("handeye_yaml").perform(context)
    if os.path.isabs(cfg_arg):
        yaml_path = cfg_arg
    else:
        yaml_path = os.path.join(
            get_package_share_directory("dummy_vision"), "config", cfg_arg
        )
    with open(yaml_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    parent = cfg.get("parent_frame", "link6_1_1")
    child = cfg.get("child_frame", "camera_link")
    xyz = cfg.get("translation", [0.0, 0.0, 0.05])
    rpy = cfg.get("rotation_rpy", [0.0, 0.0, 0.0])

    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="handeye_static_tf",
            arguments=[
                "--x",
                str(xyz[0]),
                "--y",
                str(xyz[1]),
                "--z",
                str(xyz[2]),
                "--roll",
                str(rpy[0]),
                "--pitch",
                str(rpy[1]),
                "--yaw",
                str(rpy[2]),
                "--frame-id",
                parent,
                "--child-frame-id",
                child,
            ],
        )
    )
    actions.append(
        LogInfo(msg=f"Static hand-eye TF {parent} -> {child} from {yaml_path}")
    )

    # RealSense driver already publishes camera_link -> optical; stub only when asked.
    publish_optical = LaunchConfiguration("publish_optical_stub").perform(context)
    if publish_optical == "":
        want_optical = bool(cfg.get("publish_optical_stub", False))
    else:
        want_optical = publish_optical.lower() in ("true", "1", "yes")

    if want_optical:
        o_xyz = cfg.get("optical_translation", [0.0, 0.0, 0.0])
        o_rpy = cfg.get("optical_rotation_rpy", [-1.57079632679, 0.0, -1.57079632679])
        o_child = cfg.get("optical_child_frame", "camera_color_optical_frame")
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_optical_static_tf",
                arguments=[
                    "--x",
                    str(o_xyz[0]),
                    "--y",
                    str(o_xyz[1]),
                    "--z",
                    str(o_xyz[2]),
                    "--roll",
                    str(o_rpy[0]),
                    "--pitch",
                    str(o_rpy[1]),
                    "--yaw",
                    str(o_rpy[2]),
                    "--frame-id",
                    child,
                    "--child-frame-id",
                    o_child,
                ],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_easy_handeye", default_value="false"),
            DeclareLaunchArgument("calibration_name", default_value="dummy_eih_calib"),
            DeclareLaunchArgument("handeye_yaml", default_value="handeye_static.yaml"),
            DeclareLaunchArgument(
                "publish_optical_stub",
                default_value="false",
                description="Publish camera_link->optical static TF (disable when RealSense is running)",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
