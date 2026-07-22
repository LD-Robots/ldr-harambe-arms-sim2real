"""
Full-robot simulation, complete.

Starts Gazebo with the world, bridges /clock and the IMUs, then hands over to
spawn_full_robot.launch.py for the robot itself: robot_state_publisher, the Gazebo
spawn, and the controller chain.

The split matters. `create` spawns from the /robot_description TOPIC, which only
robot_state_publisher publishes, and the controller_manager only exists once the
robot is in the world. Any launch file that runs `create` without also running RSP
waits forever on "Waiting messages on topic [robot_description]".

Usage:
    ros2 launch full_robot_gazebo full_robot_world.launch.py
    ros2 launch full_robot_gazebo full_robot_world.launch.py gui:=false
    ros2 launch full_robot_gazebo full_robot_world.launch.py controller_type:=trajectory
    ros2 launch full_robot_gazebo full_robot_world.launch.py simulation_world:=/path/to.sdf
"""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_full_robot_gazebo = FindPackageShare('full_robot_gazebo')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    declared_arguments = [
        DeclareLaunchArgument(
            'simulation_world',
            default_value=PathJoinSubstitution([FindPackageShare('gazebo_worlds'), 'worlds', 'lab.sdf']),
            description='Absolute path to the Gazebo world file',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Run Gazebo with its GUI; false runs the server only',
        ),
        DeclareLaunchArgument(
            'control_mode',
            default_value='position',
            description='Control mode: position or effort',
        ),
        DeclareLaunchArgument(
            'controller_type',
            default_value='direct',
            description="Controller type: 'direct' for JointGroupPositionController, "
                        "'trajectory' for JointTrajectoryController",
        ),
        DeclareLaunchArgument(
            'fixed_base',
            default_value='false',
            description='Fix robot base to world frame (no floating)',
        ),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y position'),
        DeclareLaunchArgument(
            'z',
            default_value='0.855',
            description='Spawn Z position, high enough to keep the robot above ground',
        ),
    ]

    world = LaunchConfiguration('simulation_world')

    # Resolve package:// and model:// URIs for every package the URDF and world reference.
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            os.path.join(get_package_prefix('full_robot_description'), 'share'),
            os.path.join(get_package_prefix('hand_description'), 'share'),
            os.path.join(get_package_prefix('imu_description'), 'share'),
            os.path.join(get_package_prefix('camera_description'), 'share'),
            os.path.join(get_package_prefix('gazebo_worlds'), 'share', 'gazebo_worlds', 'models'),
        ]),
    )

    # gz_sim.launch.py takes gz_args as one string, so the GUI and server variants
    # have to be separate includes rather than a substitution inside the argument.
    # Both pass -r: the controller_manager lives inside the Gazebo process and only
    # ticks while the sim runs, so a paused start times the spawners out.
    gz_sim_source = PythonLaunchDescriptionSource(
        PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    )
    gazebo_gui = IncludeLaunchDescription(
        gz_sim_source,
        launch_arguments={'gz_args': [world, ' -r']}.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )
    gazebo_headless = IncludeLaunchDescription(
        gz_sim_source,
        launch_arguments={'gz_args': [world, ' -r -s']}.items(),
        condition=UnlessCondition(LaunchConfiguration('gui')),
    )

    # /clock, so everything downstream can run on simulation time, plus the two IMUs.
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([pkg_full_robot_gazebo, 'config', 'bridge.yaml'])
        }],
        output='screen',
    )

    # robot_state_publisher + create + joint_state_broadcaster + the controllers.
    spawn_full_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_full_robot_gazebo, 'launch', 'spawn_full_robot.launch.py'])
        ),
        launch_arguments={
            'control_mode': LaunchConfiguration('control_mode'),
            'controller_type': LaunchConfiguration('controller_type'),
            'fixed_base': LaunchConfiguration('fixed_base'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [
        gz_resource_path,
        gazebo_gui,
        gazebo_headless,
        gz_bridge,
        spawn_full_robot,
    ])
