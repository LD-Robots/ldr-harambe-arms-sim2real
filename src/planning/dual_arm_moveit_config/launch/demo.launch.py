#!/usr/bin/env python3
"""
MoveIt2 Demo Launch for dual arm system.

Launches MoveIt planning server and RViz together.

Usage:
    ros2 launch dual_arm_moveit_config demo.launch.py
    ros2 launch dual_arm_moveit_config demo.launch.py use_rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    log_level = LaunchConfiguration("log_level")

    moveit_config = (
        MoveItConfigsBuilder("dual_arm_description", package_name="dual_arm_moveit_config")
        .robot_description(file_path="config/dual_arm_description.urdf.xacro")
        .robot_description_semantic(file_path="config/dual_arm_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": use_sim_time}],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
            {
                "octomap_frame": "urdf_base",
                "octomap_resolution": 0.05,
                "max_range": 5.0,
            },
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("dual_arm_moveit_config"), "config", "moveit.rviz"]
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher,
            move_group,
            rviz,
        ]
    )
