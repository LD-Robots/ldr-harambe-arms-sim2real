"""
All-in-one launch: hardware + homing + CST mode switch + policy controller.

Sequence:
  t=0s   : arm_real_bringup robot_group:=full
           (controller_manager + EtherCAT + mode_controller + trajectory +
            effort controllers + homing)
  t=delay: deactivate trajectory controllers
  t=+2s  : publish CST mode (10) to /mode_controller/commands
  t=+4s  : spawn harambe_policy_controller (claims effort interfaces)

Usage:
  ros2 launch harambe_policy_controller harambe_full.launch.py \\
      onnx_path:=/home/nano/policies/policy.onnx \\
      urdf_path:=/home/nano/.../full_robot_isaaclab.urdf

  # Homing takes longer than default 15s? Bump the delay:
  ros2 launch harambe_policy_controller harambe_full.launch.py homing_delay:=25.0

If runtime CST switch still fails on your drives, the fallback is to set
mode_of_operation=10 in the per-joint EtherCAT slave YAMLs so drives boot
directly in CST (skip homing).
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


CONTROLLER_NAME = "harambe_policy_controller"
CONTROLLER_TYPE = "harambe_policy_controller/HarambePolicyController"
NUM_JOINTS = 25
CST_MODE_DATA = "{data: [" + ",".join(["10"] * NUM_JOINTS) + "]}"

TRAJECTORY_CONTROLLERS = [
    "left_arm_trajectory_controller",
    "right_arm_trajectory_controller",
    "waist_controller",
    "left_leg_trajectory_controller",
    "right_leg_trajectory_controller",
]


def _build_merged_policy_yaml(context):
    """Merge YAML config with onnx_path / urdf_path overrides and dump to /tmp."""
    config_file = os.path.join(
        get_package_share_directory("harambe_policy_controller"),
        "config", "harambe_policy_controller.yaml",
    )
    onnx_path = LaunchConfiguration("onnx_path").perform(context)
    urdf_path = LaunchConfiguration("urdf_path").perform(context)

    with open(config_file, "r") as f:
        cfg = yaml.safe_load(f)

    ctrl = cfg.setdefault(CONTROLLER_NAME, {}).setdefault("ros__parameters", {})
    if onnx_path:
        ctrl["onnx_path"] = onnx_path
    if urdf_path:
        ctrl["urdf_path"] = urdf_path

    cm = cfg.setdefault("controller_manager", {}).setdefault("ros__parameters", {})
    cm.setdefault(CONTROLLER_NAME, {})["type"] = CONTROLLER_TYPE

    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml",
        prefix=f"harambe_policy_{int(time.time())}_",
        delete=False,
    )
    yaml.safe_dump(cfg, tmp)
    tmp.close()
    return tmp.name


def _build_sequence(context):
    """Construct the sequenced actions using the runtime homing_delay value."""
    delay_s = float(LaunchConfiguration("homing_delay").perform(context))

    deactivate_trajectory = ExecuteProcess(
        cmd=[
            "ros2", "control", "switch_controllers",
            "--deactivate", *TRAJECTORY_CONTROLLERS,
            "--switch-timeout", "10",
        ],
        output="screen",
    )

    switch_cst = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--once",
            "/mode_controller/commands",
            "std_msgs/msg/Float64MultiArray",
            CST_MODE_DATA,
        ],
        output="screen",
    )

    merged_yaml = _build_merged_policy_yaml(context)
    policy_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            CONTROLLER_NAME,
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
            "--param-file", merged_yaml,
            "-t", CONTROLLER_TYPE,
        ],
        output="screen",
    )

    return [
        TimerAction(period=delay_s,        actions=[deactivate_trajectory]),
        TimerAction(period=delay_s + 2.0,  actions=[switch_cst]),
        TimerAction(period=delay_s + 4.0,  actions=[policy_spawner]),
    ]


def generate_launch_description():
    arm_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("arm_real_bringup"),
                "launch",
                "arm_real.launch.py",
            ])
        ),
        launch_arguments={"robot_group": "full"}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "onnx_path",
            default_value="",
            description="Override ONNX path (empty = YAML default)",
        ),
        DeclareLaunchArgument(
            "urdf_path",
            default_value="",
            description="Override URDF path for KDL gravity comp (empty = YAML default)",
        ),
        DeclareLaunchArgument(
            "homing_delay",
            default_value="15.0",
            description="Seconds to wait after hardware bringup before starting "
                        "the CST switch + policy sequence (must exceed homing duration)",
        ),

        arm_real,

        OpaqueFunction(function=_build_sequence),
    ])
