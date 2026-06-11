from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch the Livox Mid360 driver node.

    Requires: livox_ros_driver2 — NOT available in this ROS2 environment.
    This file is provided as a migration skeleton; install livox_ros_driver2
    from https://github.com/Livox-SDK/livox_ros_driver2 before using.
    """
    return LaunchDescription([
        DeclareLaunchArgument("bd_list", default_value="100000000000000",
                              description="Livox device broadcast code list."),
        DeclareLaunchArgument("xfer_format", default_value="0"),
        DeclareLaunchArgument("publish_freq", default_value="10.0"),
        DeclareLaunchArgument("msg_frame_id", default_value="base"),

        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher2",
            output="screen",
            parameters=[
                PathJoinSubstitution([FindPackageShare("legged_perceptive_description"), "config", "mid360.json"]),
                {
                    "xfer_format": LaunchConfiguration("xfer_format"),
                    "publish_freq": LaunchConfiguration("publish_freq"),
                    "frame_id": LaunchConfiguration("msg_frame_id"),
                    "cmdline_str": LaunchConfiguration("bd_list"),
                },
            ],
        ),
    ])
