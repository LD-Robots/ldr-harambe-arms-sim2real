"""harambe_policy_legs.launch.py — RL legs policy + upper-body PVT split.

Mode-5 (PVT) split:
  * Arms + waist stay on PVT: the upper_body_pvt_controller (robot_pvt_control)
    holds / tracks the 13 upper-body joints.
  * The 12 LEG drives run the RL walking policy through this package's
    harambe_policy_legs_controller — POSITION targets + constant kp/kd. The
    EtherCAT drive closes the PD loop at 1 kHz; the host computes no torque.

The EtherCAT slave configs already lock the drives in CiA-402 mode 5
(mode_of_operation: 5, RxPDO 0x1601 with position/velocity/effort/kp/kd), so
both controllers claim {position, kp, kd} (legs) / {position, velocity, effort,
kp, kd} (upper body) on the same bus without an explicit runtime mode switch.

Sequence:
  t=0      : robot_pvt.launch.py robot_group:=upper
             (controller_manager @ 1 kHz, broadcasters, safety supervisor,
              upper_body_pvt_controller spawned INACTIVE)
  t=delay  : activate upper_body_pvt_controller (arms + waist hold)
  t=+2s    : spawn harambe_policy_legs_controller (claims the 12 leg interfaces)

The legs controller itself holds the default leg pose for `warmup_steps` policy
steps and waits for ~/enable=true before it drives the gait, so spawning it does
not immediately energise a walk.

Usage:
  ros2 launch harambe_policy_legs_controller harambe_policy_legs.launch.py \\
      onnx_path:=/home/nano/policies/policy_legs.onnx

  # Longer wait for the upper-body bring-up / homing:
  ros2 launch harambe_policy_legs_controller harambe_policy_legs.launch.py \\
      bringup_delay:=20.0
"""

import os
import tempfile
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


CONTROLLER_NAME = "harambe_policy_legs_controller"
CONTROLLER_TYPE = "harambe_policy_legs_controller/HarambePolicyLegsController"
UPPER_BODY_CONTROLLER = "upper_body_pvt_controller"


def _build_merged_yaml(context):
    """Merge the controller YAML with an onnx_path override; dump to /tmp."""
    config_file = LaunchConfiguration("config_file").perform(context)
    onnx_path = LaunchConfiguration("onnx_path").perform(context)

    if not config_file:
        config_file = os.path.join(
            get_package_share_directory(CONTROLLER_NAME),
            "config", "harambe_policy_legs_controller.yaml",
        )

    with open(config_file, "r") as f:
        cfg = yaml.safe_load(f)

    ctrl = cfg.setdefault(CONTROLLER_NAME, {}).setdefault("ros__parameters", {})
    if onnx_path:
        ctrl["onnx_path"] = onnx_path

    cm = cfg.setdefault("controller_manager", {}).setdefault("ros__parameters", {})
    cm.setdefault(CONTROLLER_NAME, {})["type"] = CONTROLLER_TYPE

    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml",
        prefix=f"harambe_policy_legs_{int(time.time())}_",
        delete=False,
    )
    yaml.safe_dump(cfg, tmp)
    tmp.close()
    return tmp.name


def _build_sequence(context):
    delay_s = float(LaunchConfiguration("bringup_delay").perform(context))

    # Activate the upper-body PVT controller (arms + waist hold on PVT). It is
    # spawned INACTIVE by robot_pvt.launch.py so the bus stays back-drivable
    # until gains are ready; here we switch it active.
    activate_upper = ExecuteProcess(
        cmd=[
            "ros2", "control", "set_controller_state",
            UPPER_BODY_CONTROLLER, "active",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    merged_yaml = _build_merged_yaml(context)
    legs_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            CONTROLLER_NAME,
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--param-file", merged_yaml,
            "-t", CONTROLLER_TYPE,
        ],
        output="screen",
    )

    return [
        TimerAction(period=delay_s,       actions=[activate_upper]),
        TimerAction(period=delay_s + 2.0, actions=[legs_spawner]),
    ]


def generate_launch_description():
    # Bring up the whole-body PVT stack for the upper body (arms + waist). The
    # legs' drives come up in mode 5 too (single EtherCAT bus); we drive them
    # with the policy controller instead of the lower_body_pvt_controller.
    robot_pvt = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robot_bringup"),
                "launch",
                "robot_pvt.launch.py",
            ])
        ),
        launch_arguments={
            "robot_group": "upper",
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "onnx_path",
            default_value=os.path.join(
                get_package_share_directory("harambe_policy_legs_controller"),
                "models", "policy.onnx"),
            description="ONNX path for the legs policy "
                        "(default = bundled models/policy.onnx; override to use another).",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="Optional custom controller YAML "
                        "(empty = installed default).",
        ),
        DeclareLaunchArgument(
            "bringup_delay",
            default_value="10.0",
            description="Seconds to wait after the PVT bring-up before "
                        "activating the upper-body controller and spawning the "
                        "legs policy controller (must exceed the upper-body "
                        "bring-up duration).",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            choices=["true", "false"],
            description="Forwarded to robot_pvt.launch.py.",
        ),

        robot_pvt,

        OpaqueFunction(function=_build_sequence),
    ])
