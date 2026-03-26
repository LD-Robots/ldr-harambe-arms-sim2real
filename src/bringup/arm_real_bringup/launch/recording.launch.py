from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler, TimerAction, IncludeLaunchDescription
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Gravity-compensated recording mode using CST (torque) EtherCAT mode.

    Drives boot directly into CST (mode 10) via ethercat_gravcomp/ slave configs.
    No runtime mode switching needed — avoids the EcCiA402Drive position override bug.

    Use motion_recorder.py in a separate terminal to record the motion.

    Usage:
        ros2 launch arm_real_bringup recording.launch.py
    """
    pkg_dual_arm_description = FindPackageShare("dual_arm_description")
    pkg_arm_real_bringup = FindPackageShare("arm_real_bringup")
    pkg_arm_ethercat_safety = FindPackageShare("arm_ethercat_safety")

    # Robot description — gravcomp:=true selects ethercat_gravcomp/ slave configs
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_dual_arm_description, "urdf", "dual_arm.urdf.xacro"]),
        " use_sim:=false",
        " gravcomp:=true",
        " only_arms:=true",
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Effort-only controller config (no JTC, no mode_controller)
    controller_config = PathJoinSubstitution(
        [pkg_arm_real_bringup, "config", "controllers_gravcomp.yaml"]
    )

    # Robot State Publisher — publishes /robot_description and TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    # ros2_control_node — loads EthercatDriver plugin
    # Drives boot directly into CST mode via ethercat_gravcomp/ configs
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="screen",
        remappings=[
            ("/joint_states", "/joint_states_raw"),
        ],
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--switch-timeout", "30",
            "--service-call-timeout", "60",
        ],
        output="screen",
    )

    # Effort controllers — receive gravity compensation torques
    left_arm_effort_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_effort_controller",
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
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )


    # Gravity compensation nodes — one per arm, computes gravity torques via PyKDL
    # tip_link includes hand_base_link so hand mass (~576g) is compensated
    # Tune comp_scale / torque_offset at runtime:
    #   ros2 param set /gravity_comp_left torque_offset "[0.3, 0.0, 0.0, 0.0, 0.0, 0.0]"
    gravity_comp_left = Node(
        package="arm_real_bringup",
        executable="gravity_comp_node.py",
        name="gravity_comp_left",
        output="screen",
        parameters=[{
            "joint_names": [
                "left_shoulder_pitch_joint_X6",
                "left_shoulder_roll_joint_X6",
                "left_shoulder_yaw_joint_X4",
                "left_elbow_pitch_joint_X6",
                "left_wrist_yaw_joint_X4",
                "left_wrist_roll_joint_X4",
            ],
            "joint_motor_types": ["X6", "X6", "X4", "X6", "X4", "X4"],
            "root_link": "urdf_simplified_torso",
            "tip_link": "left_hand_base_link",
            "controller_topic": "/left_arm_effort_controller/commands",
            "publish_rate": 100.0,
            "comp_scale": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            "torque_offset": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }],
    )

    # Right arm: 3 joints have effort/position factor sign mismatch in EtherCAT configs
    # (shoulder_roll, shoulder_yaw, wrist_yaw have negative position factor but positive
    #  effort factor). comp_scale = -1.0 corrects the gravity torque direction for these.
    gravity_comp_right = Node(
        package="arm_real_bringup",
        executable="gravity_comp_node.py",
        name="gravity_comp_right",
        output="screen",
        parameters=[{
            "joint_names": [
                "right_shoulder_pitch_joint_X6",
                "right_shoulder_roll_joint_X6",
                "right_shoulder_yaw_joint_X4",
                "right_elbow_pitch_joint_X6",
                "right_wrist_yaw_joint_X4",
                "right_wrist_roll_joint_X4",
            ],
            "joint_motor_types": ["X6", "X6", "X4", "X6", "X4", "X4"],
            "root_link": "urdf_simplified_torso",
            "tip_link": "right_hand_base_link",
            "controller_topic": "/right_arm_effort_controller/commands",
            "publish_rate": 100.0,
            #                    pitch  roll   yaw    pitch  yaw    roll
            "comp_scale":       [1.0,  -1.0,  -1.0,  1.0,  -1.0,  1.0],
            "torque_offset": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }],
    )

    # Chain: (10s) → JSB → effort_controller → gravity_comp
    delayed_jsb_spawner = TimerAction(
        period=10.0,
        actions=[joint_state_broadcaster_spawner],
    )

    start_effort_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                left_arm_effort_spawner,
                right_arm_effort_spawner,
            ],
        )
    )

    start_gravcomp_after_effort = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_arm_effort_spawner,
            on_exit=[gravity_comp_left, gravity_comp_right],
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

    # Safety monitor — monitors joint limits, provides e-stop service
    safety_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_arm_ethercat_safety, "launch", "safety_monitor.launch.py"])
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        delayed_jsb_spawner,
        start_effort_after_jsb,
        start_gravcomp_after_effort,
        joint_state_publisher,
        safety_launch,
        rviz,
    ])
