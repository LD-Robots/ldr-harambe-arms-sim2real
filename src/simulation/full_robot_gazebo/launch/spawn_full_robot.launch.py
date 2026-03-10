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

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--switch-timeout", "20",
            "--service-call-timeout", "60",
        ],
        output="screen",
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--switch-timeout", "20",
            "--service-call-timeout", "60",
        ],
        output="screen",
    )

    left_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_hand_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--switch-timeout", "20",
            "--service-call-timeout", "60",
        ],
        output="screen",
    )

    right_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_hand_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--switch-timeout", "20",
            "--service-call-timeout", "60",
        ],
        output="screen",
    )

    legs_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "legs_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--switch-timeout", "20",
            "--service-call-timeout", "60",
        ],
        output="screen",
    )

    # Whole body controller - spawned inactive (cannot coexist with per-group controllers)
    # Activate with: ros2 control switch_controllers --activate whole_body_controller --deactivate left_arm_controller right_arm_controller left_hand_controller right_hand_controller legs_controller
    whole_body_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "whole_body_controller",
            "--inactive",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--switch-timeout", "20",
            "--service-call-timeout", "60",
        ],
        output="screen",
    )

    # Chain: spawn_entity -> (5s) -> JSB -> left_arm -> right_arm -> left_hand -> right_hand -> legs -> whole_body (inactive)
    start_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner])],
        )
    )

    start_left_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[left_arm_controller_spawner],
        )
    )

    start_right_arm_after_left_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_arm_controller_spawner,
            on_exit=[right_arm_controller_spawner],
        )
    )

    start_left_hand_after_right_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_arm_controller_spawner,
            on_exit=[left_hand_controller_spawner],
        )
    )

    start_right_hand_after_left_hand = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_hand_controller_spawner,
            on_exit=[right_hand_controller_spawner],
        )
    )

    start_legs_after_right_hand = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_hand_controller_spawner,
            on_exit=[legs_controller_spawner],
        )
    )

    start_whole_body_after_legs = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=legs_controller_spawner,
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
        start_left_arm_after_jsb,
        start_right_arm_after_left_arm,
        start_left_hand_after_right_arm,
        start_right_hand_after_left_hand,
        start_legs_after_right_hand,
        start_whole_body_after_legs,
    ])
