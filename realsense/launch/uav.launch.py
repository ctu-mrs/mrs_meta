import os

from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

custom_config = LaunchConfiguration('custom_config')

uav_name = LaunchConfiguration('uav_name')
uav_namespace = LaunchConfiguration('uav_namespace')
initial_reset = LaunchConfiguration('initial_reset')

serial_no = LaunchConfiguration("serial_no")
json_file_path = LaunchConfiguration("json_file_path")

enable_pointcloud = LaunchConfiguration("enable_pointcloud")
enable_sync = LaunchConfiguration("enable_sync")
align_depth = LaunchConfiguration("align_depth")

fisheye_width = LaunchConfiguration("fisheye_width")
fisheye_height = LaunchConfiguration("fisheye_height")
enable_fisheye = LaunchConfiguration("enable_fisheye")

depth_width = LaunchConfiguration("depth_width")
depth_height = LaunchConfiguration("depth_height")
enable_depth = LaunchConfiguration("enable_depth")

color_width = LaunchConfiguration("color_width")
color_height = LaunchConfiguration("color_height")
enable_color = LaunchConfiguration("enable_color")

infra_width = LaunchConfiguration("infra_width")
infra_height = LaunchConfiguration("infra_height")
enable_infra1 = LaunchConfiguration("enable_infra1")
enable_infra2 = LaunchConfiguration("enable_infra2")

fisheye_fps = LaunchConfiguration("fisheye_fps")
depth_fps = LaunchConfiguration("depth_fps")
infra_fps = LaunchConfiguration("infra_fps")
color_fps = LaunchConfiguration("color_fps")
gyro_fps = LaunchConfiguration("gyro_fps")
accel_fps = LaunchConfiguration("accel_fps")

# Disable bond topics by default
bond = LaunchConfiguration("bond")
respawn = LaunchConfiguration("respawn")

def launch_setup(context):

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('mrs_realsense'),
                    'launch',
                    'rs_launch.py'
                ])
            ]),
            launch_arguments={
                'custom_config': custom_config,
                'camera_namespace': uav_name,
                'camera_name': uav_namespace,
                'initial_reset': initial_reset,
                # "profile" params are processed this way because it was a test of manipulation with params provided to the launchfile
                'rgb_camera.color_profile': str(color_width.perform(context)) + 'x' + str(color_height.perform(context)) + 'x' + str(color_fps.perform(context)),
                'depth_module.depth_profile': str(depth_width.perform(context)) + 'x' + str(depth_height.perform(context)) + 'x' + str(depth_fps.perform(context)),
                'depth_module.infra_profile': str(infra_width.perform(context)) + 'x' + str(infra_height.perform(context)) + 'x' + str(infra_fps.perform(context)),
                'serial_no': serial_no,
                'json_file_path': json_file_path,

                'enable_pointcloud': enable_pointcloud,
                'enable_sync': enable_sync,
                'align_depth': align_depth,

                'enable_depth': enable_depth,
                'enable_color': enable_color,
                'enable_infra1': enable_infra1,
                'enable_infra2': enable_infra2,

                'gyro_fps': gyro_fps,
                'accel_fps': accel_fps
            }.items()
    )]

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('custom_config',       default_value=PathJoinSubstitution([
                                                                        FindPackageShare('mrs_realsense'),
                                                                        'config',
                                                                        'custom_config.yaml'
                                                                    ])),

        DeclareLaunchArgument('uav_name',            default_value=os.environ["UAV_NAME"]),
        DeclareLaunchArgument('uav_namespace',       default_value="rgbd"),

        DeclareLaunchArgument("initial_reset",       default_value="true"),
        DeclareLaunchArgument("manager",             default_value="realsense2_camera_manager"),

        # Camera device specific arguments
        #DeclareLaunchArgument("serial_no",           default_value="'908212070378'"),
        DeclareLaunchArgument("serial_no",           default_value=""),
        DeclareLaunchArgument("json_file_path",      default_value="$(find realsense)/config/realsense-high-acc.json"),

        DeclareLaunchArgument("enable_pointcloud",   default_value="false"),
        DeclareLaunchArgument("enable_sync",         default_value="true"),
        DeclareLaunchArgument("align_depth",         default_value="true"),

        DeclareLaunchArgument("fisheye_width",       default_value="1920"),
        DeclareLaunchArgument("fisheye_height",      default_value="1080"),
        DeclareLaunchArgument("enable_fisheye",      default_value="false"),

        DeclareLaunchArgument("depth_width",         default_value="1280"),
        DeclareLaunchArgument("depth_height",        default_value="720"),
        DeclareLaunchArgument("enable_depth",        default_value="true"),

        DeclareLaunchArgument("color_width",         default_value="1920"),
        DeclareLaunchArgument("color_height",        default_value="1080"),
        DeclareLaunchArgument("enable_color",        default_value="true"),

        DeclareLaunchArgument("infra_width",         default_value="1280"),
        DeclareLaunchArgument("infra_height",        default_value="720"),
        DeclareLaunchArgument("enable_infra1",       default_value="false"),
        DeclareLaunchArgument("enable_infra2",       default_value="false"),

        DeclareLaunchArgument("fisheye_fps",         default_value="30"),
        DeclareLaunchArgument("depth_fps",           default_value="30"),
        DeclareLaunchArgument("infra_fps",           default_value="30"),
        DeclareLaunchArgument("color_fps",           default_value="30"),
        DeclareLaunchArgument("gyro_fps",            default_value="1000"),
        DeclareLaunchArgument("accel_fps",           default_value="1000"),

        # Disable bond topics by default
        DeclareLaunchArgument("bond",                default_value="false" ),
        DeclareLaunchArgument("respawn",             default_value="$(arg bond)" ),

        # we use OpaqueFunction so we have 'context' object and thus we can do more advanced stuff
        OpaqueFunction(function=launch_setup)#, kwargs = {'params' : set_configurable_parameters(configurable_parameters)})
    ])