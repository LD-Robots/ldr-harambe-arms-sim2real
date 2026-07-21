#!/usr/bin/env python3
"""
MTC pick-and-place demo - full stack.

Brings up everything needed to plan (and optionally execute) the 7-DOF
waist + left-arm pick-and-place task:

  1. Gazebo + robot + controllers, with or without the Gazebo GUI
  2. move_group, with ExecuteTaskSolutionCapability (required by MTC)
  3. RViz with the Motion Planning Tasks panel
  4. /reset_lever and /reset_robot helpers
  5. the mtc_pick_place node, via mtc_node_only.launch.py

Task arguments (object_id, pick_*, place_*, execute, spawn_*) are declared by
mtc_node_only.launch.py and pass straight through; run with --show-args for the
full list.

Usage:
    # full stack with the Gazebo GUI
    ros2 launch arm_mtc mtc_demo.launch.py

    # headless simulation
    ros2 launch arm_mtc mtc_demo.launch.py gazebo_gui:=false

    # plan and execute automatically instead of picking a solution in RViz
    ros2 launch arm_mtc mtc_demo.launch.py execute:=true

To iterate on the task without paying the bringup every time, leave this
running and use mtc_node_only.launch.py in a second terminal.
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'simulation_world',
            default_value=PathJoinSubstitution([
                FindPackageShare('arm_gazebo'), 'worlds', 'lab-mtc.sdf'
            ]),
            description='World file for the simulation (MTC scene by default)',
        ),
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='true',
            description='Run Gazebo with its GUI; false uses the headless bringup',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization',
        ),
    ]

    # ========== SIMULATION BRINGUP ==========
    # GUI path: Gazebo + controllers only; move_group is started separately below.
    gazebo_gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('arm_gazebo'), 'launch', 'arm_world.launch.py'
            ])
        ),
        launch_arguments={'world': LaunchConfiguration('simulation_world')}.items(),
        condition=IfCondition(LaunchConfiguration('gazebo_gui')),
    )

    # Headless path: arm_system_bringup starts move_group internally at ~10s,
    # so this branch must NOT launch a second one.
    headless_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('arm_system_bringup'), 'launch', 'headless.launch.py'
            ])
        ),
        launch_arguments={'world': LaunchConfiguration('simulation_world')}.items(),
        condition=UnlessCondition(LaunchConfiguration('gazebo_gui')),
    )

    move_group_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('arm_moveit_config'), 'launch', 'move_group.launch.py'
                    ])
                ),
                condition=IfCondition(LaunchConfiguration('gazebo_gui')),
            )
        ],
    )

    # ========== RVIZ ==========
    # config/mtc.rviz, NOT arm_moveit_config/config/moveit.rviz. Both carry the
    # moveit_task_constructor/Motion Planning Tasks panel, but they differ in
    # Fixed Frame: mtc.rviz uses "world", moveit.rviz uses "urdf_base". The robot
    # spawns at world z=0.85, so swapping them shifts the whole scene against the
    # grid by 0.85 m and throws away the saved viewpoint -- the collision objects
    # are unmoved relative to the robot, but the view reads as if everything
    # jumped. Keep mtc.rviz so the demo opens on the scene it was framed for.
    moveit_config = (
        MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config")
        .robot_description(file_path="config/arm_description.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner", "chomp", "ompl"])
        .to_moveit_configs()
    )

    rviz_node = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='log',
                arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('arm_mtc'), 'config', 'mtc.rviz'
                ])],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                    {'use_sim_time': True},
                ],
                condition=IfCondition(LaunchConfiguration('use_rviz')),
            )
        ],
    )

    # ========== RESET HELPERS ==========
    # Read the world name and the lever home pose from the launched world file,
    # then teleport the lever back on /reset_lever. Acts on demand (no start delay).
    reset_lever_node = Node(
        package='arm_gazebo',
        executable='reset_lever',
        name='reset_lever',
        output='screen',
        parameters=[
            {'world_file': LaunchConfiguration('simulation_world')},
            {'use_sim_time': True},
        ],
    )

    # Command the trajectory controllers back to the SRDF "home"/"open" poses
    # on /reset_robot. Acts on demand (no start delay).
    reset_robot_node = Node(
        package='arm_gazebo',
        executable='reset_robot',
        name='reset_robot',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ========== MTC NODE ==========
    # Wait for Gazebo, controllers and move_group (started at ~10s on the
    # headless path) to be up before planning.
    mtc_node_launch = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('arm_mtc'), 'launch', 'mtc_node_only.launch.py'
                    ])
                )
            )
        ],
    )

    return LaunchDescription(declared_arguments + [
        gazebo_gui_launch,   # 0s:  Gazebo GUI + controllers
        headless_launch,     # 0s:  headless bringup (Gazebo + controllers + move_group)
        reset_lever_node,    # 0s:  /reset_lever helper
        reset_robot_node,    # 0s:  /reset_robot helper
        move_group_launch,   # 8s:  move_group (GUI path only)
        rviz_node,           # 14s: RViz (optional)
        mtc_node_launch,     # 20s: MTC task node
    ])
