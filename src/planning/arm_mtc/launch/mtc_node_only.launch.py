#!/usr/bin/env python3
"""
MTC pick-and-place task node, on its own.

Starts nothing but the mtc_pick_place node. Use it to re-plan against an
already-running simulation without paying the ~20 s bringup every iteration.

This file owns the task arguments and their defaults; mtc_demo.launch.py
includes it rather than duplicating them.

Usage:
    # Terminal 1: full stack, left running
    ros2 launch arm_mtc mtc_demo.launch.py

    # Terminal 2: re-plan as often as needed
    ros2 launch arm_mtc mtc_node_only.launch.py
    ros2 launch arm_mtc mtc_node_only.launch.py pick_x:=0.40 pick_y:=0.30
    ros2 launch arm_mtc mtc_node_only.launch.py execute:=true

Prerequisites: Gazebo, controllers and move_group must already be running.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

# Object and table geometry default to the lab-mtc.sdf scene, transformed
# world -> urdf_base (the robot spawns at world z=0.85, so Z is offset by -0.85).
# (name, default, value_type, description)
TASK_ARGS = [
    ('object_id', 'target_cylinder', str, 'Collision object ID of the object to pick'),
    ('object_radius', '0.03035', float, 'Cylinder radius [m] (= Gazebo lever radius)'),
    ('object_height', '0.15', float, 'Cylinder height [m] (= Gazebo lever length)'),
    ('pick_x', '0.45', float, 'Pick X in urdf_base [m]'),
    ('pick_y', '0.35', float, 'Pick Y in urdf_base [m]'),
    ('pick_z', '0.175', float, 'Pick Z in urdf_base [m]'),
    ('place_x', '0.45', float, 'Place X in urdf_base [m]'),
    ('place_y', '-0.35', float, 'Place Y in urdf_base [m]'),
    ('place_z', '0.175', float, 'Place Z in urdf_base [m]'),
    ('execute', 'false', bool, 'Execute automatically instead of waiting for manual execution in RViz'),
    ('spawn_object', 'true', bool, 'Publish the object into the planning scene'),
    ('spawn_table', 'true', bool, 'Publish the tables into the planning scene'),
]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, _, description in TASK_ARGS
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

    task_parameters = {'use_sim_time': True}
    task_parameters.update({
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, _, value_type, _ in TASK_ARGS
    })

    mtc_node = Node(
        package='arm_mtc',
        executable='mtc_pick_place',
        name='mtc_pick_place',
        output='screen',
        parameters=[moveit_config.to_dict(), task_parameters],
    )

    return LaunchDescription(declared_arguments + [mtc_node])
