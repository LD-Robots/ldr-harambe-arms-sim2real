#!/usr/bin/env python3
"""
Dual-arm MTC pick-and-place demo - full stack.

Two single-arm pick-and-places, one per arm: the left arm carries the object to the
far table and goes home, then the right arm brings it back. They are two separate
MTC tasks -- the second is built only after the first has run, from the scene as it
actually ended up.

  1. Gazebo + robot + controllers, via dual_arm_gazebo
  2. move_group, with ExecuteTaskSolutionCapability (required by MTC)
  3. RViz with the Motion Planning Tasks panel
  4. the scene publisher, then the task node

Usage:
    ros2 launch dual_arm_mtc mtc_demo.launch.py
    ros2 launch dual_arm_mtc mtc_demo.launch.py gazebo_gui:=false
    ros2 launch dual_arm_mtc mtc_demo.launch.py execute:=true

It plans but does not execute by default: pick a solution in the Motion Planning
Tasks panel and run it from there. To re-plan without restarting the simulation,
leave this running and use mtc_node_only.launch.py in a second terminal.

Arm 2 waits for a go-ahead so it plans against the object's real position:

    ros2 topic pub --once /mtc_next_arm std_msgs/msg/Empty {}

Pass wait_for_trigger:=false to run both arms back to back instead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'simulation_world',
            default_value=PathJoinSubstitution([
                FindPackageShare('gazebo_worlds'), 'worlds', 'lab-mtc.sdf'
            ]),
            description='World file for the simulation (MTC scene by default)',
        ),
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='true',
            description='Run Gazebo with its GUI; false uses the headless server',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization',
        ),
    ]

    # ========== SIMULATION ==========
    # z=0.85 matches the single-arm setup and the Z offsets in config/mtc_task.yaml.
    # dual_arm_world.launch.py defaults to 0.855; that 5 mm difference between the
    # two robots looks accidental and is worth resolving separately.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('dual_arm_gazebo'), 'launch', 'dual_arm_world.launch.py'
            ])
        ),
        launch_arguments={
            'simulation_world': LaunchConfiguration('simulation_world'),
            'gui': LaunchConfiguration('gazebo_gui'),
            'z': '0.85',
        }.items(),
    )

    move_group = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('dual_arm_moveit_config'),
                        'launch', 'move_group.launch.py'
                    ])
                )
            )
        ],
    )

    # ========== RVIZ ==========
    # dual_arm_moveit_config's own moveit.rviz has no Motion Planning Tasks panel,
    # so borrow arm_mtc's. It is robot-agnostic apart from the per-link visibility
    # list, which RViz regenerates.
    moveit_config = (
        MoveItConfigsBuilder("dual_arm_description", package_name="dual_arm_moveit_config")
        .robot_description(file_path="config/dual_arm_description.urdf.xacro")
        .robot_description_semantic(file_path="config/dual_arm_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner", "chomp", "ompl"])
        .to_moveit_configs()
    )

    rviz = TimerAction(
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
    # Both act on demand, so they start immediately and just wait for a trigger.
    # reset_lever reads the world name and the lever's home pose out of the SDF and
    # teleports it back; it is robot-agnostic, so arm_gazebo's is used as is.
    reset_lever = Node(
        package='arm_gazebo',
        executable='reset_lever',
        name='reset_lever',
        output='screen',
        parameters=[
            {'world_file': LaunchConfiguration('simulation_world')},
            {'use_sim_time': True},
        ],
    )

    # dual_arm_gazebo's own reset_robot, not arm_gazebo's: on this robot a named
    # state spans several controllers (waist_yaw_joint_X8 sits in the arm chains but
    # has its own controller) and the SRDF has one `home` per arm group.
    reset_robot = Node(
        package='dual_arm_gazebo',
        executable='reset_robot',
        name='reset_robot',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_z': 0.85,          # matches the spawn height passed above
        }],
    )

    # ========== SCENE, THEN TASK ==========
    scene_publisher = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('dual_arm_mtc'),
                        'launch', 'publish_planning_scene.launch.py'
                    ])
                )
            )
        ],
    )

    mtc_node = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('dual_arm_mtc'),
                        'launch', 'mtc_node_only.launch.py'
                    ])
                )
            )
        ],
    )

    return LaunchDescription(declared_arguments + [
        gazebo,            # 0s:  Gazebo + robot + controllers
        reset_lever,       # 0s:  /reset_lever helper
        reset_robot,       # 0s:  /reset_robot helper
        move_group,        # 8s:  move_group
        scene_publisher,   # 12s: collision objects
        rviz,              # 14s: RViz (optional)
        mtc_node,          # 20s: MTC task
    ])
