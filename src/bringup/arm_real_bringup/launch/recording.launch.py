from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, RegisterEventHandler, TimerAction,
    IncludeLaunchDescription, OpaqueFunction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
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
            "--switch-timeout", "30",
            "--service-call-timeout", "60",
        ],
        output="screen",
        **kwargs,
    )


def _launch_setup(context):
    """Resolve robot_group and build the full launch graph."""
    robot_group = LaunchConfiguration("robot_group").perform(context)

    include_arms = robot_group in ("arms", "arms_waist", "full")
    include_waist = robot_group in ("arms_waist", "legs", "full")
    include_legs = robot_group in ("legs", "full")

    pkg_dual_arm_description = FindPackageShare("dual_arm_description")
    pkg_arm_real_bringup = FindPackageShare("arm_real_bringup")
    pkg_arm_ethercat_safety = FindPackageShare("arm_ethercat_safety")

    # Map robot_group -> xacro args
    if robot_group == "arms":
        xacro_extra = " only_arms:=true"
    elif robot_group == "arms_waist":
        xacro_extra = " only_arms:=false fixed_legs:=true"
    else:  # "legs" or "full"
        xacro_extra = " only_arms:=false fixed_legs:=false"

    # Robot description — gravcomp:=true selects ethercat_gravcomp/ slave configs
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_dual_arm_description, "urdf", "dual_arm.urdf.xacro"]),
        " use_sim:=false",
        " gravcomp:=true",
        xacro_extra,
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
    jsb_spawner = _make_spawner("joint_state_broadcaster")

    # ── Build effort controller + gravity comp lists based on robot_group ──

    effort_spawners = []
    gravity_comp_nodes = []

    if include_arms:
        effort_spawners.append(_make_spawner("left_arm_effort_controller"))
        effort_spawners.append(_make_spawner("right_arm_effort_controller"))

        # Gravity compensation — arms only
        gravity_comp_nodes.append(Node(
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
        ))

        # Right arm: 3 joints have effort/position factor sign mismatch in EtherCAT configs
        # (shoulder_roll, shoulder_yaw, wrist_yaw have negative position factor but positive
        #  effort factor). comp_scale = -1.0 corrects the gravity torque direction for these.
        gravity_comp_nodes.append(Node(
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
        ))

    if include_waist:
        effort_spawners.append(_make_spawner("waist_effort_controller"))

    if include_legs:
        effort_spawners.append(_make_spawner("left_leg_effort_controller"))
        effort_spawners.append(_make_spawner("right_leg_effort_controller"))

    # ── Assemble launch graph ──

    # Chain: Timer(10s) -> JSB -> effort controllers -> gravity comp (arms only)
    delayed_jsb_spawner = TimerAction(
        period=10.0,
        actions=[jsb_spawner],
    )

    start_effort_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=effort_spawners,
        )
    )

    nodes = [
        robot_state_publisher,
        ros2_control_node,
        delayed_jsb_spawner,
        start_effort_after_jsb,
    ]

    # Start gravity comp after the first effort controller is spawned
    if gravity_comp_nodes and effort_spawners:
        nodes.append(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=effort_spawners[0],
                on_exit=gravity_comp_nodes,
            )
        ))

    # Joint State Publisher — fills in default values for non-EtherCAT joints
    nodes.append(Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{
            "source_list": ["/joint_states_raw"],
            "rate": 30.0,
        }],
    ))

    # Safety monitor — monitors joint limits, provides e-stop service
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_arm_ethercat_safety, "launch", "safety_monitor.launch.py"])
        )
    ))

    # RViz2
    nodes.append(Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        additional_env={"OGRE_RTT_MODE": "Copy"},
    ))

    return nodes


def generate_launch_description():
    """Gravity-compensated recording mode using CST (torque) EtherCAT mode.

    Drives boot directly into CST (mode 10) via ethercat_gravcomp/ slave configs.
    No runtime mode switching needed — avoids the EcCiA402Drive position override bug.

    Arms get PyKDL gravity compensation. Waist and legs run with zero effort
    (free to move by hand).

    Use motion_recorder.py in a separate terminal to record the motion.

    Usage:
        ros2 launch arm_real_bringup recording.launch.py                       # arms only
        ros2 launch arm_real_bringup recording.launch.py robot_group:=full     # full robot
        ros2 launch arm_real_bringup recording.launch.py robot_group:=arms_waist
        ros2 launch arm_real_bringup recording.launch.py robot_group:=legs
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_group",
            default_value="arms",
            choices=["arms", "arms_waist", "legs", "full"],
            description="Which body groups to include: arms, arms_waist, legs, or full",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
