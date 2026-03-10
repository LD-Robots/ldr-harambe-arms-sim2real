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

    use_sim_time = LaunchConfiguration("use_sim_time")
    control_mode = LaunchConfiguration("control_mode")

    # Robot description from XACRO
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

    # NOTE: controller_manager is provided by Gazebo plugin (GazeboSimROS2ControlPlugin)
    # Do NOT launch ros2_control_node here when using sim

    # Controller spawners - chained sequentially
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
            "--service-call-timeout", "30",
        ],
        output="screen",
    )

    body_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "body_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--service-call-timeout", "30",
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
            "--service-call-timeout", "30",
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
            "--service-call-timeout", "30",
        ],
        output="screen",
    )

    left_leg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_leg_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--service-call-timeout", "30",
        ],
        output="screen",
    )

    right_leg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_leg_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--service-call-timeout", "30",
        ],
        output="screen",
    )

    # Chain: JSB (after 10s) -> body -> left_arm -> right_arm -> left_leg -> right_leg
    start_jsb = TimerAction(period=10.0, actions=[joint_state_broadcaster_spawner])

    start_body_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[body_controller_spawner],
        )
    )

    start_left_arm_after_body = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=body_controller_spawner,
            on_exit=[left_arm_controller_spawner],
        )
    )

    start_right_arm_after_left_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_arm_controller_spawner,
            on_exit=[right_arm_controller_spawner],
        )
    )

    start_left_leg_after_right_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_arm_controller_spawner,
            on_exit=[left_leg_controller_spawner],
        )
    )

    start_right_leg_after_left_leg = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_leg_controller_spawner,
            on_exit=[right_leg_controller_spawner],
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        control_mode_arg,

        robot_state_publisher,

        # Sequential controller chain
        start_jsb,
        start_body_after_jsb,
        start_left_arm_after_body,
        start_right_arm_after_left_arm,
        start_left_leg_after_right_arm,
        start_right_leg_after_left_leg,
    ])
