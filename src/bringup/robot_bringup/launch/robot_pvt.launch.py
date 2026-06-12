"""robot_pvt.launch.py — whole-body PVT bring-up on real EtherCAT hardware.

Brings up:
  - controller_manager with the PVT URDF (mode_of_operation: 5, RxPDO 0x1601)
  - robot_filtered_joint_state_broadcaster (replaces standard JSB; 30 Hz IIR)
  - robot_drive_status_broadcaster (telemetry: temps, bus voltage, error code)
  - robot_safety_supervisor (whole-body watchdog + e-stop / Kp derate)
  - upper_body_pvt_controller and/or lower_body_pvt_controller (INACTIVE;
    selected by robot_group, operator activates explicitly)

Order is enforced via OnProcessExit chains so PREOP→SAFEOP completes before
any controller spawner runs. The PVT controllers are spawned INACTIVE so the
drives stay in mode 5 with Kp=Kd=0 (back-drivable FREE) until the operator
ramps gains per the bench-rig protocol. robot_group=upper spawns the arms+waist
controller, lower the legs controller, full both.

The legacy CSP/CST stack (arm_real.launch.py) stays orthogonal — running this
launch does not touch arm_real_bringup's controller manager spawners.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, RegisterEventHandler, TimerAction,
    IncludeLaunchDescription, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, PathJoinSubstitution, LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _make_spawner(controller_name, inactive=False, **kwargs):
    args = [
        controller_name,
        "--controller-manager", "/controller_manager",
        "--controller-manager-timeout", "240",
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


def _launch_setup(context):
    robot_group = LaunchConfiguration("robot_group").perform(context)

    pkg_dual_arm_description = FindPackageShare("dual_arm_description")
    pkg_robot_bringup = FindPackageShare("robot_bringup")
    pkg_robot_safety = FindPackageShare("robot_safety")

    # PVT URDF always enumerates all 25 joints — the bus is always physically
    # full, and per-group scope is enforced inside the controller via the
    # `body_group` parameter (see _make_spawner below). Legs must stay
    # revolute in the kinematic tree (fixed_legs:=false), or the
    # controller_manager fails to claim their hardware interfaces.
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([
            pkg_dual_arm_description, "urdf", "dual_arm.urdf.xacro"]),
        " use_sim:=false pvt_mode:=true fixed_legs:=false",
    ])
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content, value_type=str)
    }

    controller_config = PathJoinSubstitution(
        [pkg_robot_bringup, "config", "controllers_pvt.yaml"])
    safety_limits_yaml = PathJoinSubstitution(
        [pkg_robot_safety, "config", "safety_limits.yaml"])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    # ros2_control node — standard JSB publishes /joint_states; we remap it to
    # /joint_states_raw so joint_state_publisher remains the sole publisher on
    # /joint_states. Raw (unfiltered, 1 kHz) is the primary state feed; the
    # filtered broadcaster publishes a smoothed copy on /joint_states_filtered.
    #
    # Parameter sources, in load order (later overrides earlier):
    #   1. robot_description           — the URDF
    #   2. controllers_pvt.yaml        — both split controllers + the kept
    #                                    whole-body block (each body_group="full")
    #   3. safety_limits.yaml          — supervisor + per-controller safety mirror
    #
    # Scope is selected by WHICH spawner runs (robot_group below), not by a
    # body_group override — each split controller's roster is its own scope.
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controller_config,
            safety_limits_yaml,
        ],
        output="screen",
        prefix="chrt -f 49",
        remappings=[
            ("/joint_states", "/joint_states_raw"),
        ],
    )

    # ── Spawners (PVT controllers start INACTIVE) ────────────────────────────
    # robot_group selects the body-region controllers to spawn:
    #   upper → upper_body_pvt_controller (arms + waist)
    #   lower → lower_body_pvt_controller (legs)
    #   full  → both
    # Each starts INACTIVE so the drives stay back-drivable (Kp=Kd=0) until the
    # operator ramps gains per the bench-rig protocol.
    spawn_upper = robot_group in ("upper", "full")
    spawn_lower = robot_group in ("lower", "full")

    jsb_spawner = _make_spawner("joint_state_broadcaster")
    filtered_jsb_spawner = _make_spawner("robot_filtered_joint_state_broadcaster")
    drive_status_spawner = _make_spawner("robot_drive_status_broadcaster")
    upper_spawner = _make_spawner("upper_body_pvt_controller", inactive=True)
    lower_spawner = _make_spawner("lower_body_pvt_controller", inactive=True)

    # ── Safety supervisor (independent node — same executor) ─────────────────
    safety_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_robot_safety, "launch", "safety.launch.py"])
        ),
        launch_arguments={"use_sim": "false"}.items(),
    )

    # ── Joint state aggregation (filtered raw + non-EtherCAT joints) ─────────
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{
            "source_list": ["/joint_states_raw"],
            "rate": 30.0,
        }],
    )

    # ── RViz (visual sanity check) ───────────────────────────────────────────
    # Skipped on headless nodes (e.g. the on-robot NUC) via use_rviz:=false.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        additional_env={"OGRE_RTT_MODE": "Copy"},
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    nodes = [
        robot_state_publisher,
        ros2_control_node,
        # Wait for EtherCAT PREOP→SAFEOP→OP before spawning controllers.
        TimerAction(period=4.0, actions=[jsb_spawner]),
    ]

    # Spawn chain: standard JSB → filtered JSB → drive_status → PVT (inactive).
    # Sequential so the controller-manager service queue stays single-file.
    nodes.append(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[filtered_jsb_spawner],
        )
    ))
    nodes.append(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=filtered_jsb_spawner,
            on_exit=[drive_status_spawner],
        )
    ))
    # Chain the selected PVT spawner(s) after drive_status, single-file:
    #   drive_status → [upper] → [lower]   (whichever are selected by robot_group)
    pvt_spawners = []
    if spawn_upper:
        pvt_spawners.append(upper_spawner)
    if spawn_lower:
        pvt_spawners.append(lower_spawner)
    prev = drive_status_spawner
    for spawner in pvt_spawners:
        nodes.append(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=prev,
                on_exit=[spawner],
            )
        ))
        prev = spawner

    # Safety supervisor + joint_state_publisher + RViz start in parallel with
    # the controller bring-up; the supervisor's own startup_grace_sec gates
    # breach detection until the joint feed is healthy.
    nodes.append(safety_launch)
    nodes.append(joint_state_publisher)
    nodes.append(rviz)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_group",
            default_value="upper",
            choices=["upper", "lower", "full"],
            description="Which split PVT controller(s) to spawn: upper "
                        "(upper_body_pvt_controller, arms), lower "
                        "(lower_body_pvt_controller, waist+legs), or full (both). "
                        "Default 'upper' — arms only is the least energetic first "
                        "bringup state.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            choices=["true", "false"],
            description="Launch RViz. Set to false for headless runs on the NUC.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
