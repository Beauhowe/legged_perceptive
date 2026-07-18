from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Start the terrain perception pipeline for the perceptive controller.

    Uses convex_plane_decomposition_ros's world_box_terrain node, which reads a Gazebo .world
    file directly and publishes planar_terrain — no elevation_mapping pipeline required.

    The controller itself is loaded by the legged_control sim/hw launch (controller_manager).
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file',
            description='Path to the Gazebo .world file used to generate terrain.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('convex_plane_decomposition_ros'),
                'launch',
                'world_box_terrain.launch.py',
            ])),
            launch_arguments={
                'world_file': LaunchConfiguration('world_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
    ])
