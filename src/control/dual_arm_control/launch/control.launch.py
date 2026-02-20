from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_dual_arm_description = FindPackageShare("dual_arm_description")

    # Robot description from XACRO
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_dual_arm_description, "urdf", "dual_arm.urdf.xacro"]),
        " use_sim:=true",
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    controllers = PathJoinSubstitution([
        FindPackageShare("dual_arm_control"),
        "config",
        "controllers.yaml"
    ])

    # Robot State Publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen"
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers, {"robot_description": robot_description}],
        output="screen"
    )

    # Controller spawners - chained sequentially via OnProcessExit
    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "10",
            "--switch-timeout", "20",
            "--service-call-timeout", "30",
        ],
        output="screen"
    )

    spawner_left_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "10",
            "--switch-timeout", "20",
            "--service-call-timeout", "30",
        ],
        output="screen"
    )

    spawner_right_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "10",
            "--switch-timeout", "20",
            "--service-call-timeout", "30",
        ],
        output="screen"
    )

    spawner_left_hand = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_hand_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "10",
            "--switch-timeout", "20",
            "--service-call-timeout", "30",
        ],
        output="screen"
    )

    spawner_right_hand = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_hand_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "10",
            "--switch-timeout", "20",
            "--service-call-timeout", "30",
        ],
        output="screen"
    )

    # Chain: JSB (after 6s) → left_arm → right_arm → left_hand → right_hand
    start_left_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner_jsb,
            on_exit=[spawner_left_arm],
        )
    )

    start_right_arm_after_left_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner_left_arm,
            on_exit=[spawner_right_arm],
        )
    )

    start_left_hand_after_right_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner_right_arm,
            on_exit=[spawner_left_hand],
        )
    )

    start_right_hand_after_left_hand = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner_left_hand,
            on_exit=[spawner_right_hand],
        )
    )

    return LaunchDescription([
        rsp,
        # controller_manager,

        # JSB starts after 6s delay, then chain continues sequentially
        TimerAction(period=3.0, actions=[spawner_jsb]),
        start_left_arm_after_jsb,
        start_right_arm_after_left_arm,
        start_left_hand_after_right_arm,
        start_right_hand_after_left_hand,
    ])
