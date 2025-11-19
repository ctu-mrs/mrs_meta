from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from mrs_lib.remappings_custom_config_parser import RemappingsCustomConfigParser
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
        LaunchConfiguration,
        IfElseSubstitution,
        PythonExpression,
        PathJoinSubstitution,
        EnvironmentVariable,
        )
import sys

import launch

import os

def generate_launch_description():

    ld = launch.LaunchDescription()

    uav_name = LaunchConfiguration('uav_name')

    pkg_name = "point_lio"

    this_pkg_path = get_package_share_directory(pkg_name)
    namespace='point_lio'

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value="",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
            condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
            else_value=custom_config
            )

    # #} end of custom_config

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    # #} end of custom_config

    # #{ preset

    preset = LaunchConfiguration('preset')

    ld.add_action(DeclareLaunchArgument(
        'preset',
        default_value="mid360",
        description="Sensor preset (defines which config file is loaded).",
    ))

    # #} end of preset

    # #{ topic_pc

    topic_pc = LaunchConfiguration('topic_pc')

    ld.add_action(DeclareLaunchArgument(
        'topic_pc',
        default_value="lidar/points",
        description="Standard point cloud topic.",
    ))

    # #} end of topic_pc

    # #{ topic_livox

    topic_livox = LaunchConfiguration('topic_livox')

    ld.add_action(DeclareLaunchArgument(
        'topic_livox',
        default_value="livox_lidar_publisher/points",
        description="Livox custom topic.",
    ))

    # #} end of topic_livox

    # #{ topic_imu

    topic_imu = LaunchConfiguration('topic_imu')

    ld.add_action(DeclareLaunchArgument(
        'topic_imu',
        default_value="livox_lidar_publisher/imu",
        description="IMU topic.",
    ))

    # #} end of topic_imu

    # #{ standalone

    standalone = LaunchConfiguration('standalone')

    declare_standalone = DeclareLaunchArgument(
        'standalone',
        default_value='true',
        description='Whether to start a as a standalone or load into an existing container.'
    )

    ld.add_action(declare_standalone)

    # #} end of standalone

    # #{ container_name

    container_name = LaunchConfiguration('container_name')

    declare_container_name = DeclareLaunchArgument(
        'container_name',
        default_value='',
        description='Name of an existing container to load into (if standalone is false)'
    )

    ld.add_action(declare_container_name)

    # #} end of container_name

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', "false"),
        description="Should the node subscribe to sim time?",
    ))

    # #} end of custom_config

    # #{ node

    node = ComposableNode(
        package='point_lio',
        plugin='point_lio::PointLio',
        name='point_lio',
        namespace=uav_name,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"uav_name": uav_name},
            {"config" : this_pkg_path+'/config/default.yaml'},
            {"preset" : [this_pkg_path+'/config/presets/',preset,'.yaml']},
            {'custom_config': custom_config},
        ],
        remappings=[
            # subscribers
            ('~/imu_in', topic_imu),
            ('~/pc_in', topic_pc),
            ('~/livox_in', topic_livox),
            # publishers
            ('~/odometry_out', '~/odometry'),
            ('~/cloud_registered_out', '~/cloud_registered'),
            ('~/cloud_registered_body_out', '~/cloud_registered_body'),
            ('~/laser_cloud_map_out', '~/laser_cloud_map'),
            ('~/linear_acceleration_out', '~/linear_acceleration'),
            ('~/path_out', '~/path'),
        ]
    )

    load_into_existing = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[node],
        condition=UnlessCondition(standalone)
    )

    ld.add_action(load_into_existing)

    # #} end of node

    # #{ standalone container

    standalone_container = ComposableNodeContainer(
        namespace=uav_name,
        name=namespace+'_point_lio_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output="screen",
        #prefix='xterm -e gdb -ex run --args',
        # prefix='gdb -ex run --args',
        # prefix='valgrind --tool=memcheck --leak-check=full --show-leak-kinds=all --log-file=/home/klaxalk/valgrind.txt',
        # prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],
        composable_node_descriptions=[node],
        parameters=[
            {'use_intra_process_comms': True},
            {'thread_num': os.cpu_count()},
            {'use_sim_time': use_sim_time},
        ],
    )

    ld.add_action(standalone_container)

    # #} end of standalone container

    return ld
