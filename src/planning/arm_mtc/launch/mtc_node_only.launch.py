#!/usr/bin/env python3
"""
MTC pick-and-place task node, on its own.

Starts nothing but the mtc_pick_place node. Use it to re-plan against an
already-running simulation without paying the ~20 s bringup every iteration.

Scene geometry and task tuning live in config/mtc_task.yaml, not in launch
arguments -- edit that file (or point task_config at your own copy) and re-run.
The node does not build the collision scene; planning_scene_publisher.py owns
it, and mtc_pick_place waits for the object to appear before planning.

Usage:
    # Terminal 1: full stack, left running
    ros2 launch arm_mtc mtc_demo.launch.py

    # Terminal 2: re-plan as often as needed
    ros2 launch arm_mtc mtc_node_only.launch.py
    ros2 launch arm_mtc mtc_node_only.launch.py execute:=true
    ros2 launch arm_mtc mtc_node_only.launch.py task_config:=/path/to/my_task.yaml

Prerequisites: Gazebo, controllers, move_group and the scene publisher must
already be running.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'task_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('arm_mtc'), 'config', 'mtc_task.yaml'
            ]),
            description='YAML holding the scene geometry and task tuning',
        ),
        DeclareLaunchArgument(
            'execute',
            default_value='false',
            description='Execute automatically instead of waiting for manual execution in RViz',
        ),
    ]

    # Pipeline order matches arm_moveit_config: MoveIt takes the LAST entry as the
    # RViz default, and OMPL must be it. The MTC node itself always uses OMPL --
    # mtc::solvers::PipelinePlanner defaults to the "ompl" pipeline by name.
    moveit_config = (
        MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config")
        .robot_description(file_path="config/arm_description.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")  # TRAC-IK
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner", "chomp", "ompl"])
        .to_moveit_configs()
    )

    mtc_node = Node(
        package='arm_mtc',
        executable='mtc_pick_place',
        name='mtc_pick_place',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            LaunchConfiguration('task_config'),
            {
                'use_sim_time': True,
                'execute': ParameterValue(LaunchConfiguration('execute'), value_type=bool),
            },
        ],
    )

    return LaunchDescription(declared_arguments + [mtc_node])
