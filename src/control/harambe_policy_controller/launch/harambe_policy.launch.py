"""
Launch file for harambe_policy_controller on real hardware (EtherCAT CST mode).

Assumes:
  - EtherCAT driver already loaded with controller_manager exposing:
      * position + velocity state interfaces per joint
      * effort command interface per joint
  - All 25 joints in CST mode (use mode_controller to switch: 10 = CST)
  - Dual IMU nodes publishing on /imu/pelvis and /imu/torso (sensor_msgs/Imu)
  - Optional /cmd_vel publisher (geometry_msgs/Twist)

This launch only starts the policy controller; the controller_manager and
EtherCAT driver are expected to be running from arm_real_bringup.

Usage:
  ros2 launch harambe_policy_controller harambe_policy.launch.py \\
      onnx_path:=/path/to/policy.onnx \\
      urdf_path:=/path/to/full_robot_isaaclab.urdf
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("harambe_policy_controller")

    onnx_path_arg = DeclareLaunchArgument(
        "onnx_path",
        default_value="",
        description="Absolute path to the trained ONNX policy file",
    )
    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value="",
        description="Absolute path to URDF for KDL gravity compensation (optional)",
    )
    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [pkg_share, "config", "harambe_policy_controller.yaml"]
        ),
        description="Controller YAML config",
    )

    # Spawn the controller via controller_manager spawner
    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "harambe_policy_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", LaunchConfiguration("config_file"),
        ],
        output="screen",
    )

    return LaunchDescription([
        onnx_path_arg,
        urdf_path_arg,
        config_arg,
        spawner,
    ])
