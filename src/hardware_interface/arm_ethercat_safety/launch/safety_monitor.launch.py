from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_safety = FindPackageShare("arm_ethercat_safety")

    safety_limits_config = PathJoinSubstitution(
        [pkg_safety, "config", "safety_limits.yaml"]
    )
    watchdog_config = PathJoinSubstitution(
        [pkg_safety, "config", "watchdog.yaml"]
    )
    estop_config = PathJoinSubstitution(
        [pkg_safety, "config", "estop.yaml"]
    )

    safety_monitor_node = Node(
        package="arm_ethercat_safety",
        executable="safety_monitor_node",
        name="safety_monitor",
        output="screen",
        parameters=[
            safety_limits_config,
            watchdog_config,
            estop_config,
        ],
    )

    return LaunchDescription([
        safety_monitor_node,
    ])
