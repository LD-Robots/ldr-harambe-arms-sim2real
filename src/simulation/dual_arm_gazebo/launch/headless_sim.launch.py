from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix
import os

def generate_launch_description():
    default_world = PathJoinSubstitution([
        FindPackageShare("dual_arm_gazebo"),
        "worlds",
        "lab-ldr.sdf"
    ])
    world_arg = DeclareLaunchArgument(
        'simulation_world',
        default_value=default_world,
        description='Absolute path to the Gazebo world file'
    )
    world = LaunchConfiguration('simulation_world')

    install_dir = get_package_prefix('dual_arm_description')
    install_gazebo_dir = get_package_prefix('dual_arm_gazebo')
    install_dir_hand = get_package_prefix('hand_description')
    install_full_robot_dir = get_package_prefix('full_robot_description')
    install_dir_camera = get_package_prefix('camera_description')

    # Set GZ_SIM_RESOURCE_PATH to ROS workspace for package:// URI resolution
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            os.path.join(install_dir, 'share'),
            os.path.join(install_dir_hand, 'share'),
            os.path.join(install_full_robot_dir, 'share'),
            os.path.join(install_dir_camera, 'share'),
            os.path.join(install_gazebo_dir, 'share', 'dual_arm_gazebo', 'worlds', 'models')
        ])
    )

    bridge_yaml = PathJoinSubstitution([
        FindPackageShare('dual_arm_gazebo'),
        'config',
        'bridge.yaml'
    ])

    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")
    world_file = PythonExpression(['"', world, '" if "', world, '" != "" else "', default_world, '"'])
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_ros_gz_sim,
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": [world_file, " -r -s"],  # server mode without GUI, auto-run
        }.items()
    )

    # Spawn at z=1.0: urdf_base is the waist area with all links extending
    # downward (arms to z=-0.83, legs to z=-0.6). z=1.0 keeps the lowest
    # point at z~0.17, safely above the ground plane (208 collision bodies
    # inside the ground was causing DART physics to stall).
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=[
            "-name", "dual_arm",
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.26',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # Launch ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        parameters=[{
            'config_file': bridge_yaml
        }],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gz_resource_path,
        gz_server,
        bridge,
        spawn_robot_node,
    ])
