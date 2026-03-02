"""Robot-pose-copy calibration launch file.

The robot moves through predefined poses in RViz while the user copies them.
Produces a calibration profile for the skeleton teleop node.

    ros2 launch arm_teleop robot_pose_calibration.launch.py
    ros2 launch arm_teleop robot_pose_calibration.launch.py profile_name:=andrei
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dual_arm = FindPackageShare("dual_arm_description")
    pkg_teleop = FindPackageShare("arm_teleop")

    # Launch arguments
    profile_arg = DeclareLaunchArgument(
        "profile_name", default_value="pose_calibrated",
        description="Name for the saved calibration profile")
    mirror_arg = DeclareLaunchArgument(
        "mirror_display", default_value="true",
        description="Mirror the camera display (natural mirror feel)")

    # Robot description
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_dual_arm, "urdf", "dual_arm_real.xacro"]),
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    # Joint State Publisher — subscribes to calibration node's output
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{
            "source_list": ["/skeleton_joint_states"],
            "rate": 30.0,
        }],
    )

    # Calibration node
    calibration_node = Node(
        package="arm_teleop",
        executable="robot_pose_calibration.py",
        name="robot_pose_calibration",
        output="screen",
        parameters=[{
            "profile_name": LaunchConfiguration("profile_name"),
            "mirror_display": LaunchConfiguration("mirror_display"),
            "publish_topic": "/skeleton_joint_states",
        }],
    )

    # RViz2
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d", PathJoinSubstitution([pkg_dual_arm, "config", "view_robot.rviz"]),
        ],
        additional_env={"OGRE_RTT_MODE": "Copy"},
    )

    return LaunchDescription([
        profile_arg,
        mirror_arg,
        robot_state_publisher,
        joint_state_publisher,
        calibration_node,
        rviz,
    ])
