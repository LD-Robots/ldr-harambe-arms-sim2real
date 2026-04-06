"""All-in-one launch: Gazebo (effort mode) + JTC PID + RL policy.

JTC with effort interface + internal PID at 200Hz handles the PD loop.
RL node sends position targets at 50Hz.

Usage:
  ros2 launch full_robot_control rl_walk_effort.launch.py
  ros2 launch full_robot_control rl_walk_effort.launch.py cmd_vel_x:=0.0  # stand only
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")
    pkg_full_robot_gazebo = FindPackageShare("full_robot_gazebo")
    pkg_full_robot_description = FindPackageShare("full_robot_description")
    pkg_ros_gz_bridge = FindPackageShare("ros_gz_bridge")

    default_policy = "/home/andrei/IsaacLab/logs/rsl_rl/harambe_flat/2026-04-01_13-58-55/exported/policy.onnx"

    # GZ resource path
    install_dir_full = get_package_prefix("full_robot_description")
    install_dir_hand = get_package_prefix("hand_description")
    install_dir_imu = get_package_prefix("imu_description")
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=":".join([
            os.path.join(install_dir_full, "share"),
            os.path.join(install_dir_hand, "share"),
            os.path.join(install_dir_imu, "share"),
        ]),
    )

    # Args
    policy_path_arg = DeclareLaunchArgument(
        "policy_path", default_value=default_policy)
    cmd_vel_x_arg = DeclareLaunchArgument(
        "cmd_vel_x", default_value="0.0")  # default: stand only (for debugging)

    # Robot description — EFFORT mode
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_full_robot_description, "urdf", "full_robot_gazebo.xacro"]),
        " control_mode:=effort",
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"]),
        ),
        launch_arguments={
            "gz_args": [
                PathJoinSubstitution([pkg_full_robot_gazebo, "worlds", "empty.sdf"]),
                " -r",
            ],
        }.items(),
    )

    clock_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_bridge, "launch", "clock_bridge.launch"]),
        ),
        launch_arguments={"bridge_name": "gz_clock_bridge"}.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "harambe",
            "-topic", "robot_description",
            "-x", "0", "-y", "0", "-z", "1.20",
        ],
        output="screen",
    )

    pelvis_imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="pelvis_imu_bridge",
        arguments=["/pelvis_imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU"],
        output="screen",
    )
    torso_imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="torso_imu_bridge",
        arguments=["/torso_imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU"],
        output="screen",
    )
    odom_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="odom_bridge",
        arguments=["/model/harambe/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
        output="screen",
    )

    # JTC with effort PID — spawners use wall clock (not sim_time) so they work when paused
    effort_pid_yaml = PathJoinSubstitution([
        pkg_full_robot_gazebo, "config", "controllers_effort_pid.yaml",
    ])

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
            "--param-file", effort_pid_yaml,
        ],
        output="screen",
    )

    jtc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "whole_body_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
            "--param-file", effort_pid_yaml,
        ],
        output="screen",
    )

    rl_node = Node(
        package="full_robot_control",
        executable="rl_locomotion_node.py",
        name="rl_locomotion",
        output="screen",
        parameters=[
            {"policy_path": LaunchConfiguration("policy_path")},
            {"cmd_vel_x": LaunchConfiguration("cmd_vel_x")},
            {"cmd_vel_y": 0.0},
            {"cmd_vel_yaw": 0.0},
            {"use_sim_time": True},
        ],
    )

    # Chain: spawn → JSB → JTC → RL node
    start_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[jsb_spawner],
        )
    )
    start_jtc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[jtc_spawner],
        )
    )
    start_rl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jtc_spawner,
            on_exit=[rl_node],
        )
    )

    return LaunchDescription([
        gz_resource_path,
        policy_path_arg,
        cmd_vel_x_arg,
        robot_state_publisher,
        gazebo,
        clock_bridge,
        spawn_entity,
        pelvis_imu_bridge,
        torso_imu_bridge,
        odom_bridge,
        start_jsb,
        start_jtc,
        start_rl,
    ])
