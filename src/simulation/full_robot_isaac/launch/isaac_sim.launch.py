"""Launch Isaac Sim scene with Harambe full robot and ros2_control controllers.

Usage:
    # Launch Isaac Sim + ROS controllers:
    ros2 launch full_robot_isaac isaac_sim.launch.py

    # With ONNX policy:
    ros2 launch full_robot_isaac isaac_sim.launch.py onnx:=/path/to/policy.onnx

    # Without Isaac Sim (ROS controllers only):
    ros2 launch full_robot_isaac isaac_sim.launch.py launch_isaac:=false
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, RegisterEventHandler, TimerAction,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ISAACLAB_PATH = "/home/alex/IsaacLab/isaaclab.sh"
ISAAC_SCRIPT = "/home/alex/Documents/GitHub/ldr-harambe-arms-sim2real/src/simulation/full_robot_isaac/scripts/isaac_scene.py"


def generate_launch_description():
    pkg_full_robot_description = FindPackageShare("full_robot_description")
    pkg_full_robot_isaac = FindPackageShare("full_robot_isaac")

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    launch_isaac_arg = DeclareLaunchArgument(
        "launch_isaac", default_value="true",
        description="Launch Isaac Sim GUI"
    )
    onnx_arg = DeclareLaunchArgument(
        "onnx", default_value="",
        description="Path to ONNX policy (optional)"
    )
    cmd_vel_x_arg = DeclareLaunchArgument(
        "cmd_vel_x", default_value="0.0",
        description="Forward velocity command"
    )
    obs_from_imu_arg = DeclareLaunchArgument(
        "obs_from_imu", default_value="false",
        description="Build ONNX observation from IMU-equivalent transforms"
    )
    log_imu_obs_arg = DeclareLaunchArgument(
        "log_imu_obs", default_value="false",
        description="Log IMU-related ONNX observation terms"
    )
    log_imu_obs_every_arg = DeclareLaunchArgument(
        "log_imu_obs_every", default_value="100",
        description="Periodic IMU observation logging interval in sim steps"
    )

    # Robot description from XACRO
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_full_robot_description, "urdf", "full_robot_gazebo.xacro"]),
        " control_mode:=isaac",
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Isaac Sim scene via isaaclab.sh
    isaac_sim = ExecuteProcess(
        cmd=[ISAACLAB_PATH, "-p", ISAAC_SCRIPT,
             "--onnx", LaunchConfiguration("onnx"),
             "--cmd_vel_x", LaunchConfiguration("cmd_vel_x"),
             "--obs_from_imu", LaunchConfiguration("obs_from_imu"),
             "--log_imu_obs", LaunchConfiguration("log_imu_obs"),
             "--log_imu_obs_every", LaunchConfiguration("log_imu_obs_every")],
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_isaac")),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # Controller Manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([pkg_full_robot_isaac, "config", "controllers.yaml"]),
        ],
        output="screen",
    )

    # Joint State Broadcaster
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

    # Whole Body Controller
    whole_body_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "whole_body_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
        ],
        output="screen",
    )

    # IMU bridge: Isaac UDP → ROS 2 /pelvis_imu/data
    imu_bridge = ExecuteProcess(
        cmd=["python3",
             "/home/alex/Documents/GitHub/ldr-harambe-arms-sim2real/src/simulation/full_robot_isaac/scripts/isaac_imu_bridge.py"],
        output="screen",
    )

    # Spawn controllers after delay
    delayed_jsb = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner],
    )

    start_whole_body = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[whole_body_controller_spawner],
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        launch_isaac_arg,
        onnx_arg,
        cmd_vel_x_arg,
        obs_from_imu_arg,
        log_imu_obs_arg,
        log_imu_obs_every_arg,
        isaac_sim,
        imu_bridge,
        robot_state_publisher,
        controller_manager,
        delayed_jsb,
        start_whole_body,
    ])
