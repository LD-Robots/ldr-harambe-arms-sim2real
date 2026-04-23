"""
Launch file for harambe_policy_controller on real hardware (EtherCAT CST mode).

Assumes:
  - controller_manager already running (from arm_real_bringup)
  - All 25 joints in CST mode (use mode_controller to switch: 10 = CST)
  - Dual IMU nodes publishing on /pelvis/imu, /torso/imu, /pelvis/grav, /torso/grav

Usage:
  # Default config from installed share (onnx_path + urdf_path must already be correct in YAML)
  ros2 launch harambe_policy_controller harambe_policy.launch.py

  # Override ONNX path (most common use case)
  ros2 launch harambe_policy_controller harambe_policy.launch.py \\
      onnx_path:=/home/nano/policies/latest.onnx

  # Override both paths
  ros2 launch harambe_policy_controller harambe_policy.launch.py \\
      onnx_path:=/path/to/policy.onnx \\
      urdf_path:=/path/to/full_robot_isaaclab.urdf

  # Use a completely custom config file
  ros2 launch harambe_policy_controller harambe_policy.launch.py \\
      config_file:=/tmp/my_custom.yaml
"""

import os
import tempfile
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CONTROLLER_NAME = "harambe_policy_controller"
CONTROLLER_TYPE = "harambe_policy_controller/HarambePolicyController"


def _apply_overrides(context):
    """Build the final param file: base YAML + optional onnx/urdf overrides."""
    config_file = LaunchConfiguration("config_file").perform(context)
    onnx_path = LaunchConfiguration("onnx_path").perform(context)
    urdf_path = LaunchConfiguration("urdf_path").perform(context)

    if not config_file:
        config_file = os.path.join(
            get_package_share_directory("harambe_policy_controller"),
            "config", "harambe_policy_controller.yaml",
        )

    with open(config_file, "r") as f:
        cfg = yaml.safe_load(f)

    ctrl_params = cfg.get(CONTROLLER_NAME, {}).get("ros__parameters", {})
    if onnx_path:
        ctrl_params["onnx_path"] = onnx_path
    if urdf_path:
        ctrl_params["urdf_path"] = urdf_path

    # Also make sure the controller_manager section knows our type (for
    # older/newer spawner variants that read it from here).
    cm = cfg.setdefault("controller_manager", {}).setdefault("ros__parameters", {})
    cm.setdefault(CONTROLLER_NAME, {})["type"] = CONTROLLER_TYPE

    # Write merged yaml to a temp file so spawner reads from disk
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml",
        prefix=f"harambe_policy_{int(time.time())}_",
        delete=False,
    )
    yaml.safe_dump(cfg, tmp)
    tmp.close()

    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            CONTROLLER_NAME,
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
            "--param-file", tmp.name,
            # -t explicitly tells the manager which plugin type to instantiate,
            # even if it wasn't pre-declared in the manager's own yaml.
            "-t", CONTROLLER_TYPE,
        ],
        output="screen",
    )
    return [spawner]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "onnx_path",
            default_value="",
            description="Absolute path to the trained ONNX policy "
                        "(overrides the 'onnx_path' field in the YAML config)",
        ),
        DeclareLaunchArgument(
            "urdf_path",
            default_value="",
            description="Absolute path to URDF for KDL gravity compensation "
                        "(overrides the 'urdf_path' field in the YAML config)",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="Optional: custom controller YAML config file. "
                        "If empty, uses the installed default.",
        ),
        OpaqueFunction(function=_apply_overrides),
    ])
