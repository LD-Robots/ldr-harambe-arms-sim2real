#!/usr/bin/env python3
"""
Planning scene publisher.

Owns the collision objects for the pick-and-place task (the object and the two
tables) and republishes them periodically so a late-starting move_group still
sees them. Geometry comes from the scene.* section of config/mtc_task.yaml.

Usage:
    ros2 launch arm_mtc publish_planning_scene.launch.py
    ros2 launch arm_mtc publish_planning_scene.launch.py task_config:=/path/to/my_task.yaml

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
            FindPackageShare('arm_mtc'), 'config', 'mtc_task.yaml'
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
