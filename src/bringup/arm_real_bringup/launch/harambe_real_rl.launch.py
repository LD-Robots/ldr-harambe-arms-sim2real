"""Real hardware RL deployment — ONNX policy on physical Harambe (25 DOF).

Startup sequence (drives MUST go through CSP first to enable cleanly):

  1. ros2_control_node + EtherCAT init (drives in pre-op state)
  2. JSB spawn → /joint_states publishing
  3. Position group controllers spawn (active) → claim position interface
     - EcCiA402Drive auto-enables drives in CSP mode (target_pos = actual_pos)
  4. mode_controller spawn (active) → claims mode_of_operation
  5. whole_body_controller (ImplicitActuator) spawn (INACTIVE) → registered but idle
  6. Wait 5s for drives to fully enable in CSP (CiA 402 OPERATION_ENABLED)
  7. Send CST mode (10) to all 25 joints via mode_controller
  8. Wait 1s for drives to transition CSP → CST
  9. Activate whole_body_controller → claims effort, runs PD
 10. Position controllers stay active (their commands now ignored by drives in CST)
 11. RL node starts publishing position targets

Safety:
  - cmd_vel_x defaults to 0.0 (stand only)
  - arm_ethercat_safety monitors limits and faults
  - Robot should be on harness for first tests

Usage:
  ros2 launch arm_real_bringup harambe_real_rl.launch.py
  ros2 launch arm_real_bringup harambe_real_rl.launch.py cmd_vel_x:=0.05
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# 25 joints all set to mode 10 (CST = Cyclic Synchronous Torque)
ALL_JOINTS_CST = "[10,10,10,10,10,10, 10,10,10,10,10,10, 10, 10,10,10,10,10,10, 10,10,10,10,10,10]"


def _make_spawner(name, inactive=False):
    args = [
        name,
        "--controller-manager", "/controller_manager",
        "--controller-manager-timeout", "120",
        "--switch-timeout", "20",
    ]
    if inactive:
        args.insert(1, "--inactive")
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=args,
        output="screen",
    )


def generate_launch_description():
    pkg_dual_arm_description = FindPackageShare("dual_arm_description")
    pkg_arm_real_bringup = FindPackageShare("arm_real_bringup")
    pkg_arm_ethercat_safety = FindPackageShare("arm_ethercat_safety")

    default_policy = "/home/andrei/IsaacLab/logs/rsl_rl/harambe_flat/2026-04-24_15-11-26_v30_sim2real/exported/policy.onnx"

    policy_path_arg = DeclareLaunchArgument("policy_path", default_value=default_policy)
    cmd_vel_x_arg = DeclareLaunchArgument("cmd_vel_x", default_value="0.0")
    cmd_vel_y_arg = DeclareLaunchArgument("cmd_vel_y", default_value="0.0")
    cmd_vel_yaw_arg = DeclareLaunchArgument("cmd_vel_yaw", default_value="0.0")

    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_dual_arm_description, "urdf", "dual_arm.urdf.xacro"]),
        " use_sim:=false only_arms:=false fixed_legs:=false",
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controller_config = PathJoinSubstitution(
        [pkg_arm_real_bringup, "config", "controllers_rl.yaml"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": False}],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="screen",
        prefix="chrt -f 49",
        remappings=[("/joint_states", "/joint_states_raw")],
    )

    # Spawners
    jsb_spawner = _make_spawner("joint_state_broadcaster")
    mode_spawner = _make_spawner("mode_controller")

    # Position group controllers — active during CSP startup
    pos_spawners = [
        _make_spawner("left_arm_group_controller"),
        _make_spawner("right_arm_group_controller"),
        _make_spawner("waist_controller"),
        _make_spawner("left_leg_group_controller"),
        _make_spawner("right_leg_group_controller"),
    ]

    # ImplicitActuator — INACTIVE at startup, activated after CSP→CST switch
    whole_body_spawner = _make_spawner("whole_body_controller", inactive=True)

    # Switch all 25 drives from CSP (8) to CST (10)
    set_cst_mode = ExecuteProcess(
        cmd=[
            "bash", "-c",
            f"echo '[INFO] Switching drives to CST mode (10)...' && "
            f"ros2 topic pub --once /mode_controller/commands "
            f"std_msgs/msg/Float64MultiArray "
            f"\"{{data: {ALL_JOINTS_CST}}}\""
        ],
        output="screen",
    )

    # Activate whole_body_controller after mode switch
    activate_whole_body = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "sleep 1 && echo '[INFO] Activating ImplicitActuatorController...' && "
            "ros2 control switch_controllers --activate whole_body_controller"
        ],
        output="screen",
    )

    rl_node = Node(
        package="full_robot_control",
        executable="rl_locomotion_node.py",
        name="rl_locomotion",
        output="screen",
        parameters=[
            {"policy_path": LaunchConfiguration("policy_path")},
            {"cmd_vel_x": LaunchConfiguration("cmd_vel_x")},
            {"cmd_vel_y": LaunchConfiguration("cmd_vel_y")},
            {"cmd_vel_yaw": LaunchConfiguration("cmd_vel_yaw")},
            {"use_sim_time": False},
        ],
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

    safety_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_arm_ethercat_safety, "launch", "safety_monitor.launch.py"])
        )
    )

    # ── Sequence ──
    nodes = [
        policy_path_arg, cmd_vel_x_arg, cmd_vel_y_arg, cmd_vel_yaw_arg,
        robot_state_publisher,
        ros2_control_node,
        TimerAction(period=2.0, actions=[jsb_spawner]),
    ]

    # After JSB → spawn mode_controller + all position controllers + whole_body (inactive)
    nodes.append(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[mode_spawner] + pos_spawners + [whole_body_spawner],
        )
    ))

    # After mode_controller spawned → wait 5s for drives to enable in CSP, then switch to CST
    nodes.append(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mode_spawner,
            on_exit=[TimerAction(period=5.0, actions=[set_cst_mode])],
        )
    ))

    # After CST mode set → wait 1s, then activate whole_body_controller
    nodes.append(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=set_cst_mode,
            on_exit=[activate_whole_body],
        )
    ))

    # After whole_body activated → start RL node
    nodes.append(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=activate_whole_body,
            on_exit=[TimerAction(period=2.0, actions=[rl_node])],
        )
    ))

    nodes.append(joint_state_publisher)
    nodes.append(safety_monitor)

    return LaunchDescription(nodes)
