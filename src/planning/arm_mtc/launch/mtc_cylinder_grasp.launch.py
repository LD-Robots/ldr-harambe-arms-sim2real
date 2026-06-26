from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    """
    Full-GUI launch variant. This launches:
    1. Gazebo WITH GUI + robot + controllers (arm_gazebo/arm_world.launch.py)
    2. MoveIt move_group
    3. RViz (MoveIt config)
    4. MTC pick and place demo node
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

    # ========== LAUNCH GAZEBO (with GUI) + robot + controllers ==========
    gazebo_gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('arm_gazebo'),
                'launch',
                'arm_world.launch.py'
            ])
        ),
        launch_arguments={
            'world': LaunchConfiguration('simulation_world')
        }.items()
    )

    # ========== LAUNCH MOVEIT MOVE_GROUP ==========
    # Wait for Gazebo and controllers to come up
    moveit_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('arm_moveit_config'),
                        'launch',
                        'move_group.launch.py'
                    ])
                )
            )
        ]
    )

    # ========== LAUNCH RVIZ (Optional) ==========
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('arm_moveit_config'),
        'config',
        'moveit.rviz'
    ])

    rviz_node = TimerAction(
        period=10.0,
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

    # ========== LEVER RESET HELPER ==========
    # Reads the world name + lever home pose from the launched world file, then
    # teleports the lever back on `/reset_lever`. Acts on demand (no start delay).
    reset_lever_node = Node(
        package='arm_gazebo',
        executable='reset_lever',
        name='reset_lever',
        output='screen',
        parameters=[
            {'world_file': LaunchConfiguration('simulation_world')},
            {'use_sim_time': True},
        ]
    )

    # ========== ROBOT RESET HELPER ==========
    # Commands the trajectory controllers back to the SRDF "home"/"open" poses on
    # `/reset_robot`. Acts on demand (no start delay).
    reset_robot_node = Node(
        package='arm_gazebo',
        executable='reset_robot',
        name='reset_robot',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ========== LAUNCH MTC PICK AND PLACE NODE ==========
    # Wait for full system + move_group initialization
    mtc_node = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='arm_mtc',
                executable='mtc_pick_place_cylinder',
                name='mtc_pick_place_cylinder',
                output='screen',
                parameters=[
                    moveit_config.to_dict(),
                    {'use_sim_time': True}
                ]
            )
        ]
    )

    return LaunchDescription([
        # Arguments
        simulation_world_arg,
        use_rviz_arg,

        # Launch sequence
        gazebo_gui_launch,       # 0s:  Gazebo GUI + controllers
        reset_lever_node,        # 0s:  /reset_lever helper
        reset_robot_node,        # 0s:  /reset_robot helper
        moveit_launch,           # 8s:  move_group
        rviz_node,               # 10s: RViz (optional)
        mtc_node,                # 18s: MTC demo
    ])
