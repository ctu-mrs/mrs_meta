#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_name = "mrs_uav_px4_api"
    this_pkg_path = get_package_share_directory(pkg_name)

    OLD_PX4_FW = os.getenv("OLD_PX4_FW", "false") == "true"
    PX4_IP = os.getenv("PX4_IP", "")

    if PX4_IP:
        default_fcu_url = f"udp://:14550@{PX4_IP}:14550"
    else:
        rate = 921600 if OLD_PX4_FW else 2000000
        default_fcu_url = f"/dev/pixhawk:{rate}"

    default_garmin_id = "33" if OLD_PX4_FW else "0"
    default_garmin_orientation = "0" if OLD_PX4_FW else "PITCH_270"


    uav_name = LaunchConfiguration("uav_name")
    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")
    tgt_system = LaunchConfiguration("tgt_system")
    config_yaml = LaunchConfiguration("config_yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    frame_namespace = LaunchConfiguration("frame_namespace")

    launch_arguments = [
        DeclareLaunchArgument(
            "uav_name",
            default_value=os.getenv("UAV_NAME", "uav1"),
        ),
        DeclareLaunchArgument(
            "fcu_url",
            default_value=default_fcu_url,
            description="FCU URL used by MAVROS",
        ),
        DeclareLaunchArgument(
            "gcs_url",
            default_value="tcp-l://",
            description="GCS URL used by MAVROS",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=os.getenv("USE_SIM_TIME", "false"),
            description="Whether MAVROS should use sim time",
        ),
        DeclareLaunchArgument(
            "config_yaml",
            default_value=os.path.join(this_pkg_path, "config", "mavros_px4_config.yaml"),
            description="Path to the MAVROS PX4 config YAML file",
        ),
        DeclareLaunchArgument(
            "garmin_id",
            default_value=default_garmin_id,
            description="Distance sensor ID reported by MAVROS for the Garmin, firmware-dependent",
        ),
        DeclareLaunchArgument(
            "garmin_orientation",
            default_value=default_garmin_orientation,
            description="Garmin sensor orientation enum used by MAVROS, firmware-dependent",
        ),
        DeclareLaunchArgument(
            "tgt_system",
            default_value="1",
            description="Target system ID for MAVROS",
        ),
        DeclareLaunchArgument(
            "frame_namespace",
            default_value=[uav_name, "/mavros"],
            description="Namespace prefix for frame IDs and other identifiers mavros owns",
        ),
    ]

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        namespace=[uav_name, "/mavros"],
        output="screen",
        parameters=[
            {"fcu_url": fcu_url},
            {"gcs_url": gcs_url},
            {"tgt_system": tgt_system},
            {"tgt_component": 1},
            {"fcu_protocol": "v2.0"},
            {"use_sim_time": use_sim_time},
            {"base_link_frame_id": [frame_namespace, "/base_link"]},
            {"odom_frame_id": [frame_namespace, "/odom"]},
            {"map_frame_id": [frame_namespace, "/map"]},
            ParameterFile(this_pkg_path + "/config/mavros_plugins.yaml", allow_substs=True),
            ParameterFile(config_yaml, allow_substs=True),
        ],
        remappings=[
            ("/diagnostics", "diagnostics"),
            (["/uas", tgt_system, "/mavlink_source"], "mavlink/source"),
            (["/uas", tgt_system, "/mavlink_sink"], "mavlink/sink"),
        ],
    )

    return LaunchDescription(
        launch_arguments
        + [
            mavros_node
        ]
    )
