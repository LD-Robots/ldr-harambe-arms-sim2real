"""
Full-robot simulation without the Gazebo GUI.

Thin wrapper over full_robot_world.launch.py. Everything -- world, resource paths,
clock and IMU bridges, robot spawn, controllers -- lives there; this file only pins
gui:=false so there is one implementation to keep correct rather than two that drift.

Every other argument still works, because an included launch description shares the
parent's context: anything set on the command line here reaches full_robot_world.

Usage:
    ros2 launch full_robot_gazebo headless_sim.launch.py
    ros2 launch full_robot_gazebo headless_sim.launch.py controller_type:=trajectory
    ros2 launch full_robot_gazebo headless_sim.launch.py simulation_world:=/path/to.sdf
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('full_robot_gazebo'), 'launch', 'full_robot_world.launch.py'
            ])
        ),
        launch_arguments={'gui': 'false'}.items(),
    )

    return LaunchDescription([world_launch])
