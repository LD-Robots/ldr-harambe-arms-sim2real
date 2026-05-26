"""robot_safety — launches the whole-body safety supervisor.

Standalone:
  ros2 launch robot_safety safety.launch.py                # real hardware
  ros2 launch robot_safety safety.launch.py use_sim:=true  # simulation

Also included by robot_pvt.launch.py (which forwards use_sim). Simulation has
no drive temperature interfaces, so temperature monitoring is disabled there.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_safety')
    limits_yaml = os.path.join(pkg_share, 'config', 'safety_limits.yaml')

    use_sim = LaunchConfiguration('use_sim')

    supervisor = Node(
        package='robot_safety',
        executable='robot_safety_supervisor',
        name='robot_safety_supervisor',
        output='screen',
        parameters=[
            limits_yaml,
            {
                'use_sim_time': use_sim,
                # Gazebo doesn't expose motor / drive temperatures or bus
                # voltage; disable temperature monitoring there so the
                # supervisor doesn't latch a STARTUP-still e-stop.
                'monitor_temperature': PythonExpression(
                    ["'", use_sim, "' == 'false'"]),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim', default_value='false',
            description='true in Gazebo: disables temperature monitoring.'),
        supervisor,
    ])
