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
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    """
    Launch file that integrates:
    1. Headless system (Gazebo + controllers + move_group) via arm_system_bringup/headless.launch.py
    2. Optional RViz visualization
    3. MTC pick and place demo node

    Note: headless.launch.py already starts move_group internally (at ~10s),
    so this file must NOT launch a second move_group.
    """

    # Declare launch arguments
    simulation_world_arg = DeclareLaunchArgument(
        'simulation_world',
        default_value=PathJoinSubstitution([
            FindPackageShare('arm_gazebo'), 'worlds', 'lab-mtc.sdf'
        ]),
        description='World file for the simulation (MTC scene by default)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    execute_arg = DeclareLaunchArgument(
        'execute',
        default_value='false',
        description='Execute automatically instead of waiting for manual execution in RViz'
    )

    # ========== BUILD MOVEIT CONFIGURATION (for the MTC node) ==========
    moveit_config = (
        MoveItConfigsBuilder("arm_description", package_name="arm_moveit_config")
        .robot_description(file_path="config/arm_description.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")  # TRAC-IK
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner", "chomp", "stomp"])
        .to_moveit_configs()
    )

    # ========== LAUNCH HEADLESS SYSTEM (Gazebo + controllers + move_group) ==========
    full_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('arm_system_bringup'),
                'launch',
                'headless.launch.py'
            ])
        ),
        launch_arguments={
            'world': LaunchConfiguration('simulation_world')
        }.items()
    )

    # ========== LAUNCH RVIZ (Optional) ==========
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('arm_moveit_config'),
        'config',
        'moveit.rviz'
    ])

    rviz_node = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='log',
                arguments=['-d', rviz_config_file],
                parameters=[
                    {'use_sim_time': True}
                ],
                condition=IfCondition(LaunchConfiguration('use_rviz'))
            )
        ]
    )

    # ========== LAUNCH MTC PICK AND PLACE NODE ==========
    # Wait for Gazebo, controllers and move_group (started at ~10s inside headless) to be ready
    mtc_node = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='arm_mtc',
                executable='mtc_pick_place_cylinder',
                name='mtc_pick_place_cylinder',
                output='screen',
                parameters=[
                    moveit_config.to_dict(),
                    {
                        'use_sim_time': True,
                        'execute': ParameterValue(
                            LaunchConfiguration('execute'), value_type=bool),
                    }
                ]
            )
        ]
    )

    return LaunchDescription([
        # Arguments
        simulation_world_arg,
        use_rviz_arg,
        execute_arg,

        # Launch sequence
        full_system_launch,      # 0s:  Gazebo + controllers + move_group (move_group at ~10s)
        rviz_node,               # 14s: RViz (optional)
        mtc_node,                # 20s: MTC demo
    ])
