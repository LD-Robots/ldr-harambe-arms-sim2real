from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the safe, read-only EtherCAT position reader.

    Requires root privileges for EtherLab master access.
    Run with: sudo -E bash -c "source install/setup.bash && ros2 launch ..."
    """
    pkg = FindPackageShare("diagnostic_tools")

    config = PathJoinSubstitution([pkg, "config", "position_reader.yaml"])

    position_reader = Node(
        package="diagnostic_tools",
        executable="position_reader_node",
        name="position_reader_node",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription([
        position_reader,
    ])
