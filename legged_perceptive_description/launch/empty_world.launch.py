import os
import re
import sys

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration("robot_type").perform(context)
    task_file = LaunchConfiguration("task_file").perform(context)
    reference_file = LaunchConfiguration("reference_file").perform(context)

    perceptive_desc_share = get_package_share_directory("legged_perceptive_description")
    gazebo_share = get_package_share_directory("legged_gazebo")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    # Reuse legged_controllers generated_paths helper for URDF output dir
    controllers_share = get_package_share_directory("legged_controllers")
    _launch_dir = os.path.join(controllers_share, "launch")
    if _launch_dir not in sys.path:
        sys.path.insert(0, _launch_dir)
    from generated_paths import get_generated_dir

    # Resolve task file with a perceptive-specific CppAD model folder, so the perceptive
    # auto-diff libraries (which include foot-placement / foot-collision / sphere-SDF
    # constraints) do NOT collide with the plain MPC libraries under legged_control/{robot_type}.
    generated_dir = get_generated_dir()
    cppad_model_folder = os.path.join(generated_dir, f"{robot_type}_perceptive")
    os.makedirs(cppad_model_folder, exist_ok=True)
    resolved_task_file = os.path.join(generated_dir, f"{robot_type}_perceptive_task.info")
    with open(task_file, encoding="utf-8") as src:
        content = src.read()
    content = re.sub(
        r"^(\s*modelFolderCppAd\s+).*$",
        rf"\g<1>{cppad_model_folder}",
        content,
        count=1,
        flags=re.MULTILINE,
    )
    with open(resolved_task_file, "w", encoding="utf-8") as out:
        out.write(content)
    task_file = resolved_task_file

    # Use the same xacro as the base sim launch for the given robot_type.
    # legged_perceptive_description/urdf/robot.xacro wraps legged_unitree_description
    # which is not available; instead use legged_description directly (same as p1_sim).
    description_share = get_package_share_directory("legged_description")
    xacro_subpath = os.path.join("urdf", robot_type, "robot.xacro")
    if not os.path.exists(os.path.join(description_share, xacro_subpath)):
        # fallback for robots that don't have a subdir (a1, aliengo, go1 at root urdf/)
        xacro_subpath = os.path.join("urdf", "robot.xacro")
    robot_xacro = os.path.join(description_share, xacro_subpath)

    world_file = os.path.join(gazebo_share, "worlds", "empty_world.world")
    generated_dir = get_generated_dir()
    generated_urdf = os.path.join(generated_dir, f"{robot_type}_perceptive.urdf")
    controller_params_file = os.path.join(generated_dir, f"{robot_type}_perceptive_gazebo_controllers.yaml")

    with open(controller_params_file, "w", encoding="utf-8") as f:
        yaml.safe_dump(
            {
                "controller_manager": {
                    "ros__parameters": {
                        "perceptive_controller": {"type": "legged/PerceptiveController"},
                    }
                },
                "perceptive_controller": {
                    "ros__parameters": {
                        "urdfFile": generated_urdf,
                        "taskFile": task_file,
                        "referenceFile": reference_file,
                        "imuName": "base_imu",
                    }
                },
            },
            f,
        )

    robot_doc = xacro.process_file(
        robot_xacro,
        mappings={
            "robot_type": robot_type,
            "hardware_plugin": "legged_gazebo/LeggedHWSim",
            "power_limit": "4",
            "contact_threshold": "40",
            "delay": "0.009",
            "delay_cycles": "9",
            "controller_params_file": controller_params_file,
        },
    )
    robot_description_xml = robot_doc.toprettyxml(indent="  ")
    with open(generated_urdf, "w", encoding="utf-8") as f:
        f.write(robot_description_xml)

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")),
        launch_arguments={"world": world_file, "init": "true", "factory": "true", "force_system": "true"}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")),
        condition=IfCondition(LaunchConfiguration("gui")),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_xml}],
    )
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", robot_type, "-z", "0.4", "-timeout", "180"],
        output="screen",
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "120",
                   "--service-call-timeout", "60"],
        output="screen",
    )
    perceptive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["perceptive_controller", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "120",
                   "--service-call-timeout", "300"],
        output="screen",
    )
    # Start joint_state_broadcaster only AFTER perceptive_controller spawner exits
    # (i.e. after CppAD compilation finishes), so controller_manager is not busy
    # when joint_state_broadcaster calls configure_controller.
    spawn_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[perceptive_controller_spawner])
    )
    spawn_jsb = RegisterEventHandler(
        OnProcessExit(target_action=perceptive_controller_spawner, on_exit=[joint_state_spawner])
    )

    return [
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        gzserver, gzclient, robot_state_publisher, spawn_entity, spawn_controllers, spawn_jsb,
    ]


def generate_launch_description():
    controllers_share = get_package_share_directory("legged_controllers")
    perceptive_desc_share = get_package_share_directory("legged_perceptive_description")

    return LaunchDescription([
        DeclareLaunchArgument("robot_type", default_value="a1"),
        DeclareLaunchArgument("task_file",
                              default_value=os.path.join(perceptive_desc_share, "..", "legged_perceptive_controllers", "config", "a1", "task.info")),
        DeclareLaunchArgument("reference_file",
                              default_value=os.path.join(controllers_share, "config", "a1", "reference.info")),
        DeclareLaunchArgument("gui", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
