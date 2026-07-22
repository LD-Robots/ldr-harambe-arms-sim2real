#!/usr/bin/env python3
"""
MTC pick-and-place task node on the dual-arm robot, on its own.

Runs this package's mtc_pick_place node against dual_arm_moveit_config and
config/mtc_task.yaml. It plans one task per arm: arm 1 moves the object to the far
table, arm 2 brings it back. Arm 2's task is built only after arm 1 has run.

Use it to re-plan against an already-running simulation without paying the bringup
every iteration.

Usage:
    # Terminal 1: full stack, left running
    ros2 launch dual_arm_mtc mtc_demo.launch.py

    # Terminal 2: re-plan as often as needed
    ros2 launch dual_arm_mtc mtc_node_only.launch.py
    ros2 launch dual_arm_mtc mtc_node_only.launch.py execute:=true

    # Terminal 3: release arm 2 once arm 1 has actually moved the object
    ros2 topic pub --once /mtc_next_arm std_msgs/msg/Empty {}

Use wait_for_trigger:=false to skip that pause and plan both arms in one go --
only sensible together with execute:=true.

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
                FindPackageShare('dual_arm_mtc'), 'config', 'mtc_task.yaml'
            ]),
            description='YAML holding the scene geometry and task tuning',
        ),
        DeclareLaunchArgument(
            'execute',
            default_value='false',
            description='Execute automatically instead of waiting for manual execution in RViz',
        ),
        DeclareLaunchArgument(
            'wait_for_trigger',
            default_value='true',
            description=(
                'Pause between the two arms until std_msgs/Empty arrives on '
                '/mtc_next_arm. Set false to plan and run both back to back.'
            ),
        ),
    ]

    # Pipeline order matches the rest of the workspace: MoveIt takes the LAST entry
    # as the RViz default, and OMPL must be it. The MTC node itself always uses
    # OMPL -- mtc::solvers::PipelinePlanner defaults to the "ompl" pipeline by name.
    moveit_config = (
        MoveItConfigsBuilder("dual_arm_description", package_name="dual_arm_moveit_config")
        .robot_description(file_path="config/dual_arm_description.urdf.xacro")
        .robot_description_semantic(file_path="config/dual_arm_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner", "chomp", "ompl"])
        .to_moveit_configs()
    )

    mtc_node = Node(
        package='dual_arm_mtc',
        executable='mtc_pick_place',
        name='mtc_pick_place',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            LaunchConfiguration('task_config'),
            {
                'use_sim_time': True,
                'execute': ParameterValue(LaunchConfiguration('execute'), value_type=bool),
                'wait_for_trigger': ParameterValue(
                    LaunchConfiguration('wait_for_trigger'), value_type=bool),
            },
        ],
    )

    return LaunchDescription(declared_arguments + [mtc_node])
