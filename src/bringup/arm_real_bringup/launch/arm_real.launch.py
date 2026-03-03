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

    # Robot description from XACRO (real hardware variant)
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_dual_arm_description, "urdf", "dual_arm.urdf.xacro"]),
        " use_sim:=false",
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

    # ros2_control_node — replaces Gazebo plugin for real hardware
    # This node loads the hardware interface plugin and controller manager
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="screen",
    )

    # Controller spawners — chained sequentially
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--switch-timeout", "20",
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

    # Chain: EtherCAT init (~30s) → JSB spawner → left_arm_controller spawner
    # TimerAction gives EtherCAT time to scan slaves and configure PDOs.
    # The spawner's --controller-manager-timeout handles any remaining wait.
    # Once JSB spawner exits (short-lived), left_arm_controller spawner starts.
    delayed_jsb_spawner = TimerAction(
        period=10.0,
        actions=[joint_state_broadcaster_spawner],
    )

    start_left_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[left_arm_controller_spawner],
        )
    )

    # Safety monitor
    safety_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_arm_ethercat_safety, "launch", "safety_monitor.launch.py"])
        )
    )

    # Joint State Publisher — fills in default values for non-EtherCAT joints
    # (hands, right arm) so robot_state_publisher can compute full TF tree
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{
            "source_list": ["/joint_states"],
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
        # Robot state publisher
        robot_state_publisher,

        # ros2_control with EtherCAT hardware interface
        ros2_control_node,

        # Sequential controller spawning (TimerAction → JSB → left_arm_controller)
        delayed_jsb_spawner,
        start_left_arm_after_jsb,

        # Safety system
        safety_launch,

        # Visualization
        joint_state_publisher,
        rviz,
    ])
