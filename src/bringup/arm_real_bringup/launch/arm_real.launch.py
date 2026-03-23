from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Packages
    pkg_dual_arm_description = FindPackageShare("dual_arm_description")
    pkg_arm_real_bringup = FindPackageShare("arm_real_bringup")
    pkg_arm_ethercat_safety = FindPackageShare("arm_ethercat_safety")

    # Robot description from XACRO (real hardware, both arms only)
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_dual_arm_description, "urdf", "dual_arm.urdf.xacro"]),
        " use_sim:=false",
        " only_arms:=true",
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Controller config
    controller_config = PathJoinSubstitution(
        [pkg_arm_real_bringup, "config", "controllers.yaml"]
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    # ros2_control_node — EtherCAT hardware interface
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="screen",
        prefix="chrt -f 49",
        remappings=[
            ("/joint_states", "/joint_states_raw"),
        ],
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--switch-timeout", "20",
            "--service-call-timeout", "60",
        ],
        output="screen",
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_group_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--switch-timeout", "20",
            "--service-call-timeout", "60",
        ],
        output="screen",
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_group_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--switch-timeout", "20",
            "--service-call-timeout", "60",
        ],
        output="screen",
    )

    mode_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mode_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    left_arm_effort_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_effort_controller",
            "--inactive",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    right_arm_effort_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_effort_controller",
            "--inactive",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    # Homing sequences — slowly move arms to URDF zero
    left_homing_node = Node(
        package="arm_real_bringup",
        executable="homing_sequence.py",
        name="left_arm_homing",
        output="screen",
        parameters=[{
            "controller_name": "left_arm_group_controller",
            "joint_names": [
                "left_shoulder_pitch_joint_X6",
                "left_shoulder_roll_joint_X6",
                "left_shoulder_yaw_joint_X4",
                "left_elbow_pitch_joint_X6",
                "left_wrist_yaw_joint_X4",
                "left_wrist_roll_joint_X4",
            ],
            "max_velocity": 0.2,
            "target_position": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "min_duration": 3.0,
        }],
    )
    right_homing_node = Node(
        package="arm_real_bringup",
        executable="homing_sequence.py",
        name="right_arm_homing",
        output="screen",
        parameters=[{
            "controller_name": "right_arm_group_controller",
            "joint_names": [
                "right_shoulder_pitch_joint_X6",
                "right_shoulder_roll_joint_X6",
                "right_shoulder_yaw_joint_X4",
                "right_elbow_pitch_joint_X6",
                "right_wrist_yaw_joint_X4",
                "right_wrist_roll_joint_X4",
            ],
            "max_velocity": 0.2,
            "target_position": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "min_duration": 3.0,
        }],
    )

    # Chain: EtherCAT init → JSB → controllers → homing
    delayed_jsb_spawner = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner],
    )

    start_controllers_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                mode_controller_spawner,
                left_arm_controller_spawner,
                right_arm_controller_spawner,
                left_arm_effort_spawner,
                right_arm_effort_spawner,
            ],
        )
    )

    start_left_homing = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_arm_controller_spawner,
            on_exit=[left_homing_node],
        )
    )
    start_right_homing = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_arm_controller_spawner,
            on_exit=[right_homing_node],
        )
    )

    # Safety monitor
    safety_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_arm_ethercat_safety, "launch", "safety_monitor.launch.py"])
        )
    )

    # Joint State Publisher — fills in default values for non-EtherCAT joints
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{
            "source_list": ["/joint_states_raw"],
            "rate": 30.0,
        }],
    )

    # RViz2
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        additional_env={"OGRE_RTT_MODE": "Copy"},
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        delayed_jsb_spawner,
        start_controllers_after_jsb,
        start_left_homing,
        start_right_homing,
        safety_launch,
        joint_state_publisher,
        rviz,
    ])
