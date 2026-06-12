"""View the robot in RViz from a remote machine on a given ROS_DOMAIN_ID.

Robot-less viewer: it does NOT build the URDF or run robot_state_publisher. It
just opens RViz on the chosen domain and subscribes to what the robot is already
publishing there — /robot_description (transient_local), /tf, /tf_static,
/joint_states. So run the robot (e.g. robot_pvt_viewer.launch.py) on the robot
machine on the same domain, then run this anywhere on the network.

    ros2 launch robot_bringup remote_view.launch.py            # domain 62
    ros2 launch robot_bringup remote_view.launch.py domain_id:=7

The RobotModel display reads /robot_description over the topic, so no xacro or
description package is needed on the viewing machine.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("robot_bringup"), "config", "remote_view.rviz"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "domain_id",
            default_value="62",
            description="ROS_DOMAIN_ID to view on (must match the robot's).",
        ),
        # Set the domain for the rviz2 process before it starts its DDS participant.
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("domain_id")),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
