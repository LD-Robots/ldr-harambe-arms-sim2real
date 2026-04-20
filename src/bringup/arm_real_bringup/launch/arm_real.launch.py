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


def _make_spawner(controller_name, inactive=False, **kwargs):
    """Create a controller spawner node."""
    args = [
        controller_name,
        "--controller-manager", "/controller_manager",
        "--controller-manager-timeout", "120",
        "--switch-timeout", "20",
        "--service-call-timeout", "60",
    ]
    if inactive:
        args.insert(1, "--inactive")
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=args,
        output="screen",
        **kwargs,
    )


def _make_homing(name, controller_name, joint_names, max_velocity=0.2, min_duration=3.0):
    """Create a homing sequence node."""
    return Node(
        package="arm_real_bringup",
        executable="homing_sequence.py",
        name=name,
        output="screen",
        parameters=[{
            "controller_name": controller_name,
            "joint_names": joint_names,
            "max_velocity": max_velocity,
            "target_position": [0.0] * len(joint_names),
            "min_duration": min_duration,
        }],
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

    # Map robot_group → xacro args
    if robot_group == "arms":
        xacro_extra = " only_arms:=true"
    elif robot_group == "arms_waist":
        xacro_extra = " only_arms:=false fixed_legs:=true"
    else:  # "legs" or "full"
        xacro_extra = " only_arms:=false fixed_legs:=false"

    # Robot description from XACRO (real hardware)
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_dual_arm_description, "urdf", "dual_arm.urdf.xacro"]),
        " use_sim:=false",
        xacro_extra,
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

    # Joint State Broadcaster (always needed)
    jsb_spawner = _make_spawner("joint_state_broadcaster")

    # Mode controller (always needed — ros2_control ignores joints not in HW interface)
    mode_spawner = _make_spawner("mode_controller")

    # ── Build controller + homing lists based on robot_group ──

    position_spawners = []   # active at startup (JTC)
    effort_spawners = []     # inactive at startup
    homing_nodes = []

    if include_arms:
        left_arm_spawner = _make_spawner("left_arm_trajectory_controller")
        right_arm_spawner = _make_spawner("right_arm_trajectory_controller")
        position_spawners.extend([left_arm_spawner, right_arm_spawner])

        effort_spawners.extend([
            _make_spawner("left_arm_effort_controller", inactive=True),
            _make_spawner("right_arm_effort_controller", inactive=True),
        ])

        homing_nodes.append((left_arm_spawner, _make_homing(
            "left_arm_homing", "left_arm_trajectory_controller",
            [
                "left_shoulder_pitch_joint_X6",
                "left_shoulder_roll_joint_X6",
                "left_shoulder_yaw_joint_X4",
                "left_elbow_pitch_joint_X6",
                "left_wrist_yaw_joint_X4",
                "left_wrist_roll_joint_X4",
            ],
            max_velocity=0.2,
        )))
        homing_nodes.append((right_arm_spawner, _make_homing(
            "right_arm_homing", "right_arm_trajectory_controller",
            [
                "right_shoulder_pitch_joint_X6",
                "right_shoulder_roll_joint_X6",
                "right_shoulder_yaw_joint_X4",
                "right_elbow_pitch_joint_X6",
                "right_wrist_yaw_joint_X4",
                "right_wrist_roll_joint_X4",
            ],
            max_velocity=0.2,
        )))

    if include_waist:
        waist_spawner = _make_spawner("waist_controller")
        position_spawners.append(waist_spawner)

        effort_spawners.append(
            _make_spawner("waist_effort_controller", inactive=True),
        )

        homing_nodes.append((waist_spawner, _make_homing(
            "waist_homing", "waist_controller",
            ["waist_yaw_joint_X8"],
            max_velocity=0.15,
        )))

    if include_legs:
        left_leg_spawner = _make_spawner("left_leg_trajectory_controller")
        right_leg_spawner = _make_spawner("right_leg_trajectory_controller")
        position_spawners.extend([left_leg_spawner, right_leg_spawner])

        effort_spawners.extend([
            _make_spawner("left_leg_effort_controller", inactive=True),
            _make_spawner("right_leg_effort_controller", inactive=True),
        ])

        # Leg homing disabled — motion_player homes to first waypoint instead
        # homing_nodes.append((left_leg_spawner, _make_homing(
        #     "left_leg_homing", "left_leg_trajectory_controller",
        #     [
        #         "left_hip_pitch_joint_X8",
        #         "left_hip_roll_joint_X8",
        #         "left_hip_yaw_joint_X8",
        #         "left_knee_joint_X8",
        #         "left_ankle_pitch_joint_X4",
        #         "left_ankle_roll_joint_X4",
        #     ],
        #     max_velocity=0.1,
        #     min_duration=5.0,
        # )))
        # homing_nodes.append((right_leg_spawner, _make_homing(
        #     "right_leg_homing", "right_leg_trajectory_controller",
        #     [
        #         "right_hip_pitch_joint_X8",
        #         "right_hip_roll_joint_X8",
        #         "right_hip_yaw_joint_X8",
        #         "right_knee_joint_X8",
        #         "right_ankle_roll_joint_X4",
        #         # right_ankle_pitch_joint_X4 — not connected yet (bus 24)
        #     ],
        #     max_velocity=0.1,
        #     min_duration=5.0,
        # )))

    # ── Assemble launch graph ──

    nodes = [
        robot_state_publisher,
        ros2_control_node,
        # Delay JSB spawner to let EtherCAT init complete
        TimerAction(period=2.0, actions=[jsb_spawner]),
    ]

    # After JSB exits → spawn mode + all position + all effort controllers
    nodes.append(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[mode_spawner] + position_spawners + effort_spawners,
        )
    ))

    # After each position controller spawner exits → start its homing
    for spawner, homing in homing_nodes:
        nodes.append(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawner,
                on_exit=[homing],
            )
        ))

    # Safety monitor
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_arm_ethercat_safety, "launch", "safety_monitor.launch.py"])
        )
    ))

    # Joint State Publisher — fills in non-EtherCAT joints for full TF tree
    nodes.append(Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{
            "source_list": ["/joint_states_raw"],
            "rate": 30.0,
        }],
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
    """Real hardware bringup with configurable body groups.

    Usage:
        ros2 launch arm_real_bringup arm_real.launch.py                       # arms only
        ros2 launch arm_real_bringup arm_real.launch.py robot_group:=full     # full robot
        ros2 launch arm_real_bringup arm_real.launch.py robot_group:=arms_waist
        ros2 launch arm_real_bringup arm_real.launch.py robot_group:=legs
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
