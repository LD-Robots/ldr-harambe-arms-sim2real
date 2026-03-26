from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_prefix
import os


def generate_launch_description():
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_full_robot_gazebo = FindPackageShare('full_robot_gazebo')
    pkg_ros_gz_bridge = FindPackageShare('ros_gz_bridge')

    # Set GZ_SIM_RESOURCE_PATH so Gazebo can resolve model:// URIs
    install_dir_full_robot = get_package_prefix('full_robot_description')
    install_dir_hand = get_package_prefix('hand_description')
    install_dir_imu = get_package_prefix('imu_description')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(install_dir_full_robot, 'share'),
            os.path.join(install_dir_hand, 'share'),
            os.path.join(install_dir_imu, 'share'),
        ])
    )

    # Set GZ_SIM_SYSTEM_PLUGIN_PATH so Gazebo can find custom plugins
    install_dir_policy_plugin = get_package_prefix('harambe_gz_policy_plugin')
    plugin_lib_dir = os.path.join(install_dir_policy_plugin, 'lib')
    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=plugin_lib_dir
    )

    # ONNX Runtime library path
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))))
    onnx_lib_dir = os.path.join(workspace_dir, 'third_party', 'onnxruntime', 'lib')
    onnx_ld_path = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value=':'.join([plugin_lib_dir, onnx_lib_dir, os.environ.get('LD_LIBRARY_PATH', '')])
    )

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([pkg_full_robot_gazebo, 'worlds', 'empty.sdf']),
        description='Path to the world file'
    )

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='position',
        description='Control mode: position, effort, gz_pid, or gz_policy'
    )

    onnx_path_arg = DeclareLaunchArgument(
        'onnx_path',
        default_value='',
        description='Path to ONNX policy (for gz_policy mode)'
    )

    # Launch Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r']
        }.items()
    )

    # Spawn the full robot
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_full_robot_gazebo, 'launch', 'spawn_full_robot.launch.py'])
        ),
        launch_arguments={
            'control_mode': LaunchConfiguration('control_mode'),
            'onnx_path': LaunchConfiguration('onnx_path'),
        }.items()
    )

    # Clock bridge - essential for simulation time
    clock_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_bridge, 'launch', 'clock_bridge.launch'])
        ),
        launch_arguments={
            'bridge_name': 'gz_clock_bridge'
        }.items()
    )

    # IMU bridges: Gazebo Harmonic -> ROS 2
    pelvis_imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pelvis_imu_bridge',
        arguments=['/pelvis_imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        output='screen',
    )

    torso_imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='torso_imu_bridge',
        arguments=['/torso_imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        output='screen',
    )

    return LaunchDescription([
        gz_resource_path,
        gz_plugin_path,
        onnx_ld_path,
        world_arg,
        control_mode_arg,
        onnx_path_arg,
        gazebo,
        clock_bridge,
        spawn_robot,
        pelvis_imu_bridge,
        torso_imu_bridge,
    ])
