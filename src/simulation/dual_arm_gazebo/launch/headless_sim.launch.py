"""
Dual-arm simulation without the Gazebo GUI.

Thin wrapper over dual_arm_world.launch.py. Everything -- world, resource paths,
clock bridge, robot spawn, controllers -- lives there; this file only pins gui:=false
so there is one implementation to keep correct rather than two that drift.

Previously this file started Gazebo and `create` but no robot_state_publisher, so
/robot_description was never published and it hung on
"Waiting messages on topic [robot_description]" without ever spawning the robot or
starting a controller.

Usage:
    ros2 launch dual_arm_gazebo headless_sim.launch.py
    ros2 launch dual_arm_gazebo headless_sim.launch.py simulation_world:=/path/to.sdf
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
                FindPackageShare('dual_arm_gazebo'), 'launch', 'dual_arm_world.launch.py'
            ])
        ),
        launch_arguments={'gui': 'false'}.items(),
    )

    return LaunchDescription([world_launch])
