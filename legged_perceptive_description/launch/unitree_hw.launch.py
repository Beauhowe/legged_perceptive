import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration("robot_type").perform(context)

    perceptive_desc_share = get_package_share_directory("legged_perceptive_description")
    robot_xacro = os.path.join(perceptive_desc_share, "urdf", "robot.xacro")

    robot_doc = xacro.process_file(robot_xacro, mappings={"robot_type": robot_type, "sim": "false"})
    robot_description_xml = robot_doc.toprettyxml(indent="  ")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_xml}],
    )

    # NOTE: legged_unitree_hw node — package not available in this environment.
    # Install from the legged_control hardware repo before using.
    legged_hw_node = Node(
        package="legged_unitree_hw",
        executable="legged_unitree_hw",
        name="legged_unitree_hw",
        output="screen",
        parameters=[
            PathJoinSubstitution([FindPackageShare("legged_unitree_hw"), "config", f"{robot_type}.yaml"]),
            {"robot_type": robot_type},
        ],
    )

    # NOTE: mid360 launch — requires livox_ros_driver2 (not in this environment).
    mid360_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("legged_perceptive_description"), "launch", "mid360.launch.py"])
        ),
    )

    # NOTE: realsense2_camera (tracking T265) — not available in this environment.
    # Uncomment and adapt once realsense2_camera ROS2 is installed.
    # tracking_camera_node = Node(
    #     package="realsense2_camera",
    #     executable="realsense2_camera_node",
    #     namespace="tracking_camera",
    #     parameters=[{"device_type": "t265", "enable_pose": True, "publish_odom_tf": False}],
    # )

    return [robot_state_publisher, legged_hw_node, mid360_launch]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_type", default_value="a1",
                              description="Robot type: [a1, aliengo, go1, laikago]"),
        OpaqueFunction(function=launch_setup),
    ])
