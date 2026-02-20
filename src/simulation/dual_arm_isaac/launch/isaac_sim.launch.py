#!/usr/bin/env python3
"""
Isaac Sim ROS2 bridge launch for dual arm system.

Starts the ROS2 side: robot_state_publisher, ros2_control_node, and controllers.
Isaac Sim must be started separately with the robot loaded and publishing on
/isaac_joint_states and subscribing to /isaac_joint_commands.

Usage:
    1. Start Isaac Sim with the dual arm robot USD scene
    2. ros2 launch dual_arm_isaac isaac_sim.launch.py
    3. ros2 launch dual_arm_moveit_config move_group.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Packages
    pkg_dual_arm_description = FindPackageShare("dual_arm_description")
    pkg_dual_arm_isaac = FindPackageShare("dual_arm_isaac")

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use Isaac Sim time"
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info", description="Log level"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    # Robot description from XACRO (Isaac variant)
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_dual_arm_description, "urdf", "dual_arm.urdf.xacro"]),
        " use_isaac:=true",
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Controllers config
    controllers_yaml = PathJoinSubstitution([
        pkg_dual_arm_isaac, "config", "controllers.yaml"
    ])

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # ros2_control_node (standalone - Isaac Sim communicates via topics)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            controllers_yaml,
            {"use_sim_time": use_sim_time},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Controller spawners - chained sequentially
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    left_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_hand_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    right_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_hand_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # Chain: JSB → left_arm → right_arm → left_hand → right_hand
    start_left_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[left_arm_controller_spawner],
        )
    )

    start_right_arm_after_left_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_arm_controller_spawner,
            on_exit=[right_arm_controller_spawner],
        )
    )

    start_left_hand_after_right_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_arm_controller_spawner,
            on_exit=[left_hand_controller_spawner],
        )
    )

    start_right_hand_after_left_hand = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_hand_controller_spawner,
            on_exit=[right_hand_controller_spawner],
        )
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        log_level_arg,

        # Core nodes
        robot_state_publisher,
        ros2_control_node,

        # Controller spawning chain
        joint_state_broadcaster_spawner,
        start_left_arm_after_jsb,
        start_right_arm_after_left_arm,
        start_left_hand_after_right_arm,
        start_right_hand_after_left_hand,
    ])
