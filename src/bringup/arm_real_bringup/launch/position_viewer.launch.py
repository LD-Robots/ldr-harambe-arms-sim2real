from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Read-only position viewer using ICube ethercat_driver_ros2.

    Launches ros2_control with the EthercatDriver plugin but drives stay
    disabled (auto_state_transitions: false in readonly configs).
    Only joint_state_broadcaster is spawned — no motion controller.

    Usage:
        ros2 launch arm_real_bringup position_viewer.launch.py
    """
    pkg_dual_arm_description = FindPackageShare("dual_arm_description")
    pkg_arm_real_bringup = FindPackageShare("arm_real_bringup")

    # Robot description — real hardware URDF with readonly EtherCAT configs
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_dual_arm_description, "urdf", "dual_arm.urdf.xacro"]),
        " use_sim:=false",
        " readonly:=true",
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Viewer-only controller config (joint_state_broadcaster only)
    controller_config = PathJoinSubstitution(
        [pkg_arm_real_bringup, "config", "controllers_viewer.yaml"]
    )

    # Robot State Publisher — publishes /robot_description and TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    # ros2_control_node — loads EthercatDriver plugin, runs PDO exchange
    # Drives stay in SWITCH_ON_DISABLED (readonly configs disable auto state transitions)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="screen",
    )

    # Joint State Broadcaster — publishes /joint_states from EtherCAT TxPDO data
    # EtherCAT init (slave scan, PDO config, state transitions) takes ~30s,
    # controller_manager services only start after hardware is fully ready
    joint_state_broadcaster_spawner = TimerAction(
        period=10.0,
        actions=[Node(
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
        )],
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
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        joint_state_publisher,
        rviz,
    ])
