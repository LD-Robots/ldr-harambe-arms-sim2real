#!/usr/bin/env python3
"""
Planning scene publisher for the dual-arm MTC task.

Runs arm_mtc's planning_scene_publisher.py against this package's config, so the
task node and the scene read the same numbers. Applies the collision objects once
through /apply_planning_scene rather than re-publishing on a topic.

Usage:
    ros2 launch dual_arm_mtc publish_planning_scene.launch.py

Prerequisites: move_group must be running.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    task_config_arg = DeclareLaunchArgument(
        'task_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('dual_arm_mtc'), 'config', 'mtc_task.yaml'
        ]),
        description='YAML holding the scene geometry and task tuning',
    )

    publisher_node = Node(
        package='arm_mtc',
        executable='planning_scene_publisher.py',
        name='planning_scene_publisher',
        output='screen',
        parameters=[
            LaunchConfiguration('task_config'),
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([task_config_arg, publisher_node])
