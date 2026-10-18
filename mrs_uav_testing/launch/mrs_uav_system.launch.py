#!/usr/bin/env python3

import launch
import os
import sys

from launch.actions import IncludeLaunchDescription, GroupAction, SetEnvironmentVariable, DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
        LaunchConfiguration,
        IfElseSubstitution,
        PythonExpression,
        PathJoinSubstitution,
        EnvironmentVariable,
        )

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "mrs_uav_testing"
    this_pkg_path = get_package_share_directory(pkg_name)

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    # #} end of custom_config

    # #{ run_automatic_start

    run_automatic_start = LaunchConfiguration('run_automatic_start')

    declare_run_automatic_start = DeclareLaunchArgument(
        'run_automatic_start',
        default_value='false',
        description='Whether to start the automatic start node.'
    )

    ld.add_action(declare_run_automatic_start)

    # #} end of standalorun_automatic_start

    # #{ standalone

    standalone = LaunchConfiguration('standalone')

    declare_standalone = DeclareLaunchArgument(
        'standalone',
        default_value='false',
        description='Whether to start in standalone mode.'
    )

    ld.add_action(declare_standalone)

    # #} end of standalorun_automatic_start

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value=this_pkg_path + "/config/default_custom_config.yaml",
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

    # #{ platform_config

    platform_config = LaunchConfiguration('platform_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'platform_config',
        default_value=get_package_share_directory("mrs_multirotor_simulator")+"/config/mrs_uav_system/x500.yaml",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        ))

    # behaviour:
    #     platform_config == "" => platform_config: ""
    #     platform_config == "/<path>" => platform_config: "/<path>"
    #     platform_config == "<path>" => platform_config: "$(pwd)/<path>"
    platform_config = IfElseSubstitution(
            condition=PythonExpression(['"', platform_config, '" != "" and ', 'not "', platform_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), platform_config]),
            else_value=platform_config
            )

    # #} end of platform_config

    # #{ world_config

    world_config = LaunchConfiguration('world_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'world_config',
        default_value=this_pkg_path + "/config/default_world_config.yaml",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        ))

    # behaviour:
    #     world_config == "" => world_config: ""
    #     world_config == "/<path>" => world_config: "/<path>"
    #     world_config == "<path>" => world_config: "$(pwd)/<path>"
    world_config = IfElseSubstitution(
            condition=PythonExpression(['"', world_config, '" != "" and ', 'not "', world_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), world_config]),
            else_value=world_config
            )

    # #} end of world_config

    # #{ network_config

    network_config = LaunchConfiguration('network_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'network_config',
        default_value=this_pkg_path + "/config/default_network_config.yaml",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        ))

    # behaviour:
    #     network_config == "" => network_config: ""
    #     network_config == "/<path>" => network_config: "/<path>"
    #     network_config == "<path>" => network_config: "$(pwd)/<path>"
    network_config = IfElseSubstitution(
            condition=PythonExpression(['"', network_config, '" != "" and ', 'not "', network_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), network_config]),
            else_value=network_config
            )

    # #} end of network_config

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value="true",
        description="Should the node subscribe to sim time?",
    ))

    # #} end of custom_config

    ld.add_action(
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('mrs_uav_autostart'), '/launch/automatic_start.launch.py'
                ]),
                launch_arguments={
                    'custom_config': custom_config,
                    'uav_name': uav_name,
                    'use_sim_time': use_sim_time,
                }.items(),
                condition=IfCondition(run_automatic_start)
            )
        ])
    )

    ld.add_action(
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('mrs_uav_core'), '/launch/core.launch.py'
                ]),
                launch_arguments={
                    'uav_name': uav_name,
                    'standalone': standalone,
                    'use_sim_time': use_sim_time,
                    'custom_config': custom_config,
                    'platform_config': platform_config,
                    'world_config': world_config,
                    'network_config': network_config,
                }.items()
            )
        ])
    )

    return ld
