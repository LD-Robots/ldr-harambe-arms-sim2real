"""robot_pvt_policy.launch.py — PVT bring-up, but the LEGS run the RL policy.

A duplicate of robot_pvt.launch.py with ONE swap: instead of
`lower_body_pvt_controller`, the 12 leg joints are driven by
`harambe_policy_legs_controller` (the RL walking policy, position-target output
into the same mode-5 drive PD). Arms + waist stay on `upper_body_pvt_controller`.

Brings up:
  - controller_manager with the PVT URDF (mode_of_operation: 5, RxPDO 0x1601)
  - robot_filtered_joint_state_broadcaster + robot_drive_status_broadcaster
  - robot_safety_supervisor (whole-body watchdog)
  - upper_body_pvt_controller (arms+waist) and/or harambe_policy_legs_controller
    (legs), selected by robot_group.

robot_group:
  upper → upper_body_pvt_controller (arms + waist)
  lower → harambe_policy_legs_controller (legs, RL policy)
  full  → both

By default (start_active:=true) the body controllers spawn ACTIVE: the policy
legs controller activates immediately, ramps the legs current→default pose over
warmup, then HOLDS the default and waits for `~/enable=true` before walking (it
does NOT walk on its own). Pass start_active:=false to spawn INACTIVE (drives
back-drivable, Kp=Kd=0) and activate by hand. The policy ONNX defaults to the
model bundled in the controller package (models/policy.onnx); override with
onnx_path:=<path>.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
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

POLICY_CONTROLLER = "harambe_policy_legs_controller"
POLICY_TYPE = "harambe_policy_legs_controller/HarambePolicyLegsController"


def _make_spawner(controller_name, inactive=False, extra_args=None, **kwargs):
    args = [
        controller_name,
        "--controller-manager", "/controller_manager",
        "--controller-manager-timeout", "240",
        "--switch-timeout", "20",
        "--service-call-timeout", "60",
    ]
    if extra_args:
        args += extra_args
    if inactive:
        args.insert(1, "--inactive")
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=args,
        output="screen",
        **kwargs,
    )


def _policy_config_path(context):
    """Build the legs-policy controller config (type + params + resolved
    onnx_path) and dump it to a temp file. This is loaded INTO the
    controller_manager (ros2_control_node) at startup — same pattern as
    controllers_pvt.yaml — so the manager knows the controller's type. The
    Jazzy spawner does NOT accept -t / a controller-type arg, so the type MUST
    come from the controller_manager's own params, not the spawner."""
    pkg_share = get_package_share_directory(POLICY_CONTROLLER)
    config_file = os.path.join(pkg_share, "config", f"{POLICY_CONTROLLER}.yaml")
    onnx_path = LaunchConfiguration("onnx_path").perform(context)
    if not onnx_path:
        onnx_path = os.path.join(pkg_share, "models", "policy.onnx")

    with open(config_file, "r") as f:
        cfg = yaml.safe_load(f)
    params = cfg.setdefault(POLICY_CONTROLLER, {}).setdefault("ros__parameters", {})
    params["onnx_path"] = onnx_path
    # log_csv:=true → the controller publishes ~/debug so obs_csv_logger can write
    # the sim-format CSV.
    if LaunchConfiguration("log_csv").perform(context) == "true":
        params["publish_debug"] = True
    # dry_run:=true → the policy runs + logs but the motors HOLD the default pose
    # (policy target is never sent to the drives). Safe shadow mode.
    if LaunchConfiguration("dry_run").perform(context) == "true":
        params["dry_run"] = True
    # Register the controller type in the controller_manager block.
    cm = cfg.setdefault("controller_manager", {}).setdefault("ros__parameters", {})
    cm.setdefault(POLICY_CONTROLLER, {})["type"] = POLICY_TYPE

    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", prefix="harambe_policy_legs_", delete=False)
    yaml.safe_dump(cfg, tmp)
    tmp.close()
    return tmp.name


def _launch_setup(context):
    robot_group = LaunchConfiguration("robot_group").perform(context)
    # start_active=true (default) spawns the body controllers ACTIVE: the legs
    # policy controller activates immediately (ramps current->default over warmup,
    # then HOLDS default and waits for ~/enable before walking — it does NOT walk
    # on its own). start_active=false keeps the old behavior (spawn INACTIVE,
    # drives back-drivable, operator activates by hand).
    start_active = LaunchConfiguration("start_active").perform(context) == "true"
    inactive = not start_active

    pkg_dual_arm_description = FindPackageShare("dual_arm_description")
    pkg_robot_bringup = FindPackageShare("robot_bringup")
    pkg_robot_safety = FindPackageShare("robot_safety")

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

    # ros2_control node — controllers_pvt.yaml registers the PVT controllers +
    # broadcasters; the legs-policy config (type + params + onnx) is added so the
    # controller_manager knows harambe_policy_legs_controller's type at startup
    # (the spawner then loads it by name — the Jazzy spawner has no -t arg).
    policy_config = _policy_config_path(context)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controller_config,
            safety_limits_yaml,
            policy_config,
        ],
        output="screen",
        prefix="chrt -f 49",
        remappings=[
            ("/joint_states", "/joint_states_raw"),
        ],
    )

    # ── Spawners (ALL controllers start INACTIVE) ────────────────────────────
    #   upper → upper_body_pvt_controller (arms + waist)
    #   lower → harambe_policy_legs_controller (legs, RL policy)
    #   full  → both
    spawn_upper = robot_group in ("upper", "full")
    spawn_lower = robot_group in ("lower", "full")

    jsb_spawner = _make_spawner("joint_state_broadcaster")
    filtered_jsb_spawner = _make_spawner("robot_filtered_joint_state_broadcaster")
    drive_status_spawner = _make_spawner("robot_drive_status_broadcaster")
    upper_spawner = _make_spawner("upper_body_pvt_controller", inactive=inactive)
    # SWAP: legs run the RL policy controller instead of lower_body_pvt_controller.
    # Type + params come from policy_config loaded into the controller_manager
    # above, so this spawns by name like the others (no -t, no --param-file).
    legs_spawner = _make_spawner(POLICY_CONTROLLER, inactive=inactive)

    # Optional CSV logger — subscribes to the controller's ~/debug and writes the
    # SAME obs CSV format as the sims (play_mjx / play_isaac / Gazebo plugin), so a
    # real run drops into scripts/plot_csv_gui.py next to the sim CSVs.
    csv_logger = Node(
        package=POLICY_CONTROLLER,
        executable="obs_csv_logger.py",
        output="screen",
        parameters=[{
            "csv_path": LaunchConfiguration("csv_path"),
            "debug_topic": f"/{POLICY_CONTROLLER}/debug",
        }],
        condition=IfCondition(LaunchConfiguration("log_csv")),
    )

    safety_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_robot_safety, "launch", "safety.launch.py"])
        ),
        launch_arguments={"use_sim": "false"}.items(),
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{
            "source_list": ["/joint_states_raw"],
            "rate": 30.0,
        }],
    )

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

    # Spawn chain: JSB → filtered JSB → drive_status → [upper] → [legs policy].
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
    body_spawners = []
    if spawn_upper:
        body_spawners.append(upper_spawner)
    if spawn_lower:
        body_spawners.append(legs_spawner)
    prev = drive_status_spawner
    for spawner in body_spawners:
        nodes.append(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=prev,
                on_exit=[spawner],
            )
        ))
        prev = spawner

    nodes.append(safety_launch)
    nodes.append(joint_state_publisher)
    nodes.append(rviz)
    nodes.append(csv_logger)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_group",
            default_value="upper",
            choices=["upper", "lower", "full"],
            description="Which controller(s) to spawn: upper "
                        "(upper_body_pvt_controller, arms+waist), lower "
                        "(harambe_policy_legs_controller, legs RL policy), or "
                        "full (both). Default 'upper'.",
        ),
        DeclareLaunchArgument(
            "onnx_path",
            default_value="",
            description="Legs policy ONNX (empty = the model bundled in "
                        "harambe_policy_legs_controller/models/policy.onnx).",
        ),
        DeclareLaunchArgument(
            "start_active",
            default_value="true",
            choices=["true", "false"],
            description="true (default): spawn the body controllers ACTIVE — the "
                        "legs policy activates immediately, ramps to the default "
                        "pose and HOLDS it waiting for ~/enable (does NOT walk on "
                        "its own). false: spawn INACTIVE (drives back-drivable), "
                        "operator activates by hand.",
        ),
        DeclareLaunchArgument(
            "dry_run",
            default_value="false",
            choices=["true", "false"],
            description="true: SHADOW mode — the policy runs and logs (~/debug, "
                        "CSV) but the motors HOLD the default pose; the policy "
                        "target is never sent to the drives. Safe way to validate "
                        "the obs pipeline + action sanity on hardware. Pair with "
                        "log_csv:=true.",
        ),
        DeclareLaunchArgument(
            "log_csv",
            default_value="true",
            choices=["true", "false"],
            description="true (default): publish ~/debug and run obs_csv_logger, "
                        "writing the same obs CSV format as the sims (play_mjx/"
                        "play_isaac/Gazebo) — drops into scripts/plot_csv_gui.py. "
                        "Pass log_csv:=false to disable.",
        ),
        DeclareLaunchArgument(
            "csv_path",
            default_value="",
            description="Output path for the obs CSV. Empty (default) → "
                        "/tmp/csv_real/obs_real_<timestamp>.csv (unique per run); "
                        "a directory → <dir>/obs_real_<timestamp>.csv; a *.csv "
                        "file → used verbatim.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            choices=["true", "false"],
            description="Launch RViz. Set false for headless runs on the NUC.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
