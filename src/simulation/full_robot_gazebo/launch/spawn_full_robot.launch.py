from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_full_robot_description = FindPackageShare("full_robot_description")

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )
    control_mode_arg = DeclareLaunchArgument(
        "control_mode", default_value="position",
        description="Control mode: position or effort"
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0", description="X position")
    y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Y position")
    z_arg = DeclareLaunchArgument("z", default_value="1.26", description="Z position")

    use_sim_time = LaunchConfiguration("use_sim_time")
    control_mode = LaunchConfiguration("control_mode")
    x_pos = LaunchConfiguration("x")
    y_pos = LaunchConfiguration("y")
    z_pos = LaunchConfiguration("z")

    # Robot description from XACRO (with control_mode arg)
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_full_robot_description, "urdf", "full_robot_gazebo.xacro"]),
        " control_mode:=", control_mode,
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "harambe",
            "-topic", "robot_description",
            "-x", x_pos,
            "-y", y_pos,
            "-z", z_pos,
        ],
        output="screen",
    )

    # Controller spawners - chained sequentially via OnProcessExit events
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
            "--switch-timeout", "30",
            "--service-call-timeout", "60",
        ],
        output="screen",
    )

    # Single whole_body_controller for RL policy deployment
    whole_body_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "whole_body_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--switch-timeout", "20",
            "--service-call-timeout", "60",
        ],
        output="screen",
    )

    # Chain: spawn_entity -> (5s) -> JSB -> whole_body_controller
    start_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner])],
        )
    )

    start_whole_body_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[whole_body_controller_spawner],
        )
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg, control_mode_arg, x_arg, y_arg, z_arg,

        # RSP first, then spawn entity
        robot_state_publisher,
        spawn_entity,

        # Sequential controller chain (triggered by events)
        start_jsb_after_spawn,
        start_whole_body_after_jsb,
    ])
