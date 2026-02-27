"""Skeleton tracking teleop + Robot State Publisher + RViz2.

Launch the full RealSense skeleton teleop stack:
    ros2 launch arm_teleop realsense_skeleton_teleop.launch.py
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
    smoothing_arg = DeclareLaunchArgument(
        "smoothing_alpha", default_value="0.3",
        description="EMA smoothing (0=max smoothing, 1=none)")
    gain_arg = DeclareLaunchArgument(
        "gain", default_value="1.0",
        description="Global gain multiplier")
    mirror_arg = DeclareLaunchArgument(
        "mirror_display", default_value="true",
        description="Mirror the debug display (natural mirror feel)")
    calibration_arg = DeclareLaunchArgument(
        "calibration_profile", default_value="",
        description="Calibration profile name or path (empty = uncalibrated)")
    auto_calibrate_arg = DeclareLaunchArgument(
        "auto_calibrate", default_value="false",
        description="Run calibration routine on startup")

    # Robot description (real hardware URDF, no Gazebo plugins)
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_dual_arm, "urdf", "dual_arm_real.xacro"]),
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Robot State Publisher — /joint_states → TF for RViz
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    # Joint State Publisher — merges skeleton joints with defaults for
    # uncontrolled joints (hands), publishes to /joint_states
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{
            "source_list": ["/skeleton_joint_states"],
            "rate": 30.0,
        }],
    )

    # Skeleton teleop node
    skeleton_teleop = Node(
        package="arm_teleop",
        executable="realsense_skeleton_teleop.py",
        name="realsense_skeleton_teleop",
        output="screen",
        parameters=[{
            "smoothing_alpha": LaunchConfiguration("smoothing_alpha"),
            "gain": LaunchConfiguration("gain"),
            "mirror_display": LaunchConfiguration("mirror_display"),
            "publish_rate": 30.0,
            "enabled": True,
            "show_debug_window": True,
            "min_detection_confidence": 0.7,
            "min_tracking_confidence": 0.5,
            "publish_topic": "/skeleton_joint_states",
            "calibration_profile": LaunchConfiguration("calibration_profile"),
            "auto_calibrate": LaunchConfiguration("auto_calibrate"),
        }],
    )

    # RViz2
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d", PathJoinSubstitution([pkg_teleop, "config", "skeleton_teleop.rviz"]),
        ],
        additional_env={"OGRE_RTT_MODE": "Copy"},
    )

    return LaunchDescription([
        smoothing_arg,
        gain_arg,
        mirror_arg,
        calibration_arg,
        auto_calibrate_arg,
        robot_state_publisher,
        joint_state_publisher,
        skeleton_teleop,
        rviz,
    ])
