"""robot_pvt_viewer.launch.py — whole-body PVT read-only viewer.

Parallel to arm_real_bringup/position_viewer.launch.py but for the PVT stack.
Brings up the EtherCAT bus with the ethercat_pvt_readonly/ slave configs so
drives stay in SWITCH_ON_DISABLED — no enable transition, no torque output.
Only the broadcasters spawn; robot_pvt_controller is NOT loaded.

Use this for:
  - inspecting live joint positions while back-driving the robot by hand
  - pre-flight checks before robot_pvt.launch.py
  - calibration sessions (combine with demo_joint_offset)

Drives can be enabled out from under this viewer ONLY by re-flashing the slave
configs or relaunching with robot_pvt.launch.py — the read-only YAMLs have
auto_state_transitions: false / auto_fault_reset: false.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, RegisterEventHandler, TimerAction,
    IncludeLaunchDescription, OpaqueFunction,
)
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

    # Resolve the readonly slave config dir to an absolute path so the xacro
    # arg substitution lands as a literal string, not a launch substitution
    # object (xacro args are plain strings, no late binding).
    readonly_dir = PathJoinSubstitution(
        [pkg_robot_bringup, "config", "ethercat_pvt_readonly"]).perform(context)

    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([
            pkg_dual_arm_description, "urdf", "dual_arm.urdf.xacro"]),
        # Both flags explicit -- see robot_pvt.launch.py for why implicit is unsafe.
        " use_sim:=false pvt_mode:=true readonly:=true fixed_legs:=false fixed_waist:=false",
        " pvt_yaml_dir:=", readonly_dir,
    ])
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content, value_type=str)
    }

    controller_config = PathJoinSubstitution(
        [pkg_robot_bringup, "config", "controllers_pvt_viewer.yaml"])
    safety_limits_yaml = PathJoinSubstitution(
        [pkg_robot_safety, "config", "safety_limits.yaml"])
    body_group_override = PathJoinSubstitution(
        [pkg_robot_bringup, "config", "body_groups", robot_group + ".yaml"])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    # Standard joint_state_broadcaster publishes to /joint_states; remap so
    # joint_state_publisher remains the sole publisher on /joint_states and
    # the EtherCAT feed lands on /joint_states_raw (1 kHz, unfiltered).
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controller_config,
            safety_limits_yaml,
            body_group_override,
        ],
        output="screen",
        prefix="chrt -f 49",
        remappings=[
            ("/joint_states", "/joint_states_raw"),
        ],
    )

    # ── Spawners — broadcasters only, no robot_pvt_controller ────────────────
    jsb_spawner = _make_spawner("joint_state_broadcaster")
    filtered_jsb_spawner = _make_spawner("robot_filtered_joint_state_broadcaster")
    drive_status_spawner = _make_spawner("robot_drive_status_broadcaster")

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
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        additional_env={"OGRE_RTT_MODE": "Copy"},
    )

    nodes = [
        robot_state_publisher,
        ros2_control_node,
        # Wait for EtherCAT PREOP→SAFEOP transition before spawning controllers.
        # Drives stay SWITCH_ON_DISABLED in readonly mode (no OP transition).
        TimerAction(period=4.0, actions=[jsb_spawner]),
    ]

    # Spawn order: standard JSB (raw, /joint_states_raw) → filtered JSB
    # (/joint_states_filtered) → drive_status. Sequential so the controller-
    # manager service queue stays single-file during startup.
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

    nodes.append(safety_launch)
    nodes.append(joint_state_publisher)
    nodes.append(rviz)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_group",
            default_value="arms",
            choices=["arms", "arms_waist", "legs", "full"],
            description="Which body groups to include: arms, arms_waist, legs, or full",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
