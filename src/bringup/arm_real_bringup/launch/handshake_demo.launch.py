"""
Handshake demo launch.

Pre-reqs:
  - arm_real.launch.py is already running (controllers active).
  - The inspire ROS 2 workspace is sourced and the inspire hand stack is
    running (publishes /inspire/<side>/state, serves /inspire/<side>/run_action).

Examples:
  ros2 launch arm_real_bringup handshake_demo.launch.py
  ros2 launch arm_real_bringup handshake_demo.launch.py mode:=oneshot
  ros2 launch arm_real_bringup handshake_demo.launch.py pose_source:=current
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_side = LaunchConfiguration("arm_side")
    pose_source = LaunchConfiguration("pose_source")
    mode = LaunchConfiguration("mode")
    config = LaunchConfiguration("config")

    config_path = PathJoinSubstitution(
        [FindPackageShare("arm_real_bringup"), "config", config]
    )

    handshake_node = Node(
        package="arm_real_bringup",
        executable="handshake_demo.py",
        name="handshake_demo",
        output="screen",
        emulate_tty=True,
        parameters=[
            config_path,
            {
                "arm_side": arm_side,
                "pose_source": pose_source,
                "mode": mode,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("arm_side", default_value="right",
                              description="'right' or 'left'"),
        DeclareLaunchArgument("pose_source", default_value="predefined",
                              description="'predefined' (use config) or 'current' (snapshot /joint_states)"),
        DeclareLaunchArgument("mode", default_value="continuous",
                              description="'continuous' or 'oneshot'"),
        DeclareLaunchArgument("config", default_value="handshake.yaml",
                              description="Parameter YAML in arm_real_bringup/config"),
        handshake_node,
    ])
