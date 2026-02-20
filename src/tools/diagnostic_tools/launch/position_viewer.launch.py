from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """EtherCAT position reader + Robot State Publisher + RViz2.

    Launches everything in one command:
        ros2 launch diagnostic_tools position_viewer.launch.py

    Requires udev rule for non-root EtherCAT access:
        echo 'KERNEL=="EtherCAT[0-9]*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-ethercat.rules
    """
    pkg_diag = FindPackageShare("diagnostic_tools")
    pkg_dual_arm = FindPackageShare("dual_arm_description")

    # Robot description (real hardware URDF, no Gazebo plugins)
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_dual_arm, "urdf", "dual_arm_real.xacro"]),
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Position reader — EtherCAT exchange at 1 kHz, publishes /joint_states
    position_reader = Node(
        package="diagnostic_tools",
        executable="position_reader_node",
        name="position_reader_node",
        output="screen",
        parameters=[PathJoinSubstitution([pkg_diag, "config", "position_reader.yaml"])],
    )

    # Robot State Publisher — converts /joint_states to TF for RViz
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    # Joint State Publisher — publishes default (zero) values for joints
    # not covered by the position reader (hands, right arm)
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{
            "source_list": ["/joint_states"],
            "rate": 30.0,
        }],
    )

    # RViz2
    # In RViz: set Fixed Frame to "world", then Add > RobotModel (topic: /robot_description)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        additional_env={"OGRE_RTT_MODE": "Copy"},
    )

    return LaunchDescription([
        position_reader,
        robot_state_publisher,
        joint_state_publisher,
        rviz,
    ])
