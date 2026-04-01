"""Effort PD control for dual arms on real hardware.

Boots drives in CST (torque) mode via gravcomp slave configs, then uses
JointTrajectoryController with effort command interface and physics-based
PD gains. The controller internally computes:
    torque = Kp * (target_pos - actual_pos) - Kd * actual_vel

This is the sim2real equivalent of Isaac Lab's ImplicitActuator —
same PD gains, same effort interface, but on real EtherCAT hardware.

Usage:
    ros2 launch arm_real_bringup effort_pd.launch.py
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _make_spawner(controller_name, **kwargs):
    """Create a controller spawner node."""
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            controller_name,
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--switch-timeout", "20",
            "--service-call-timeout", "60",
        ],
        output="screen",
        **kwargs,
    )


def generate_launch_description():

    pkg_dual_arm_description = FindPackageShare("dual_arm_description")
    pkg_arm_real_bringup = FindPackageShare("arm_real_bringup")
    pkg_arm_ethercat_safety = FindPackageShare("arm_ethercat_safety")

    # Robot description — CST mode via gravcomp:=true, dual arm only
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([
            pkg_dual_arm_description, "urdf", "dual_arm.urdf.xacro",
        ]),
        " use_sim:=false",
        " gravcomp:=true",
        " only_arms:=true",
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Effort PD controller config
    controller_config = PathJoinSubstitution(
        [pkg_arm_real_bringup, "config", "controllers_effort_pd.yaml"]
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
    # Drives boot directly into CST mode via ethercat_gravcomp/ slave configs
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="screen",
        remappings=[
            ("/joint_states", "/joint_states_raw"),
        ],
    )

    # Controller spawners
    jsb_spawner = _make_spawner("joint_state_broadcaster")
    effort_pd_spawner = _make_spawner("dual_arm_effort_controller")

    # Chain: Timer(10s) → JSB → effort PD controller
    delayed_jsb = TimerAction(period=10.0, actions=[jsb_spawner])

    start_effort_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[effort_pd_spawner],
        )
    )

    nodes = [
        robot_state_publisher,
        ros2_control_node,
        delayed_jsb,
        start_effort_after_jsb,

        # Joint State Publisher — fills in non-EtherCAT joints for full TF tree
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[{
                "source_list": ["/joint_states_raw"],
                "rate": 30.0,
            }],
        ),

        # Safety monitor
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_arm_ethercat_safety, "launch", "safety_monitor.launch.py",
                ])
            )
        ),

        # RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            additional_env={"OGRE_RTT_MODE": "Copy"},
        ),
    ]

    return nodes
