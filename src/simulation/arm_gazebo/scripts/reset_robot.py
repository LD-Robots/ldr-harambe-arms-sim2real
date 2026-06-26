#!/usr/bin/env python3
"""
Reset the robot back to its upright home state in Gazebo.

Two things can be wrong after a run:
  1. The arm/hand joints are somewhere off (left from the last motion).
  2. The whole robot has TIPPED OVER — its base isn't fixed to the world (only the
     arm/hand joints are driven; the legs are passive), so the arm swinging can
     topple it.

So /reset_robot does two things, in order:
  A. Teleport the robot model back to its upright spawn pose via the Gazebo
     UserCommands `set_pose` service (stands a fallen robot back up).
  B. Command the trajectory controllers back to the SRDF "home"/"open" poses.

Trigger:
    ros2 topic pub --once /reset_robot std_msgs/msg/Empty {}

Joint targets are read from the MoveIt SRDF <group_state> entries ("home"/"open"),
the same poses MoveIt uses, so they stay in sync if those change.

Parameters
----------
  reset_base      : bool  teleport the base upright          (true)
  model           : str   Gazebo model name                  (arm)
  world           : str   Gazebo world name ("" -> auto)     ("")
  base_x/y/z      : float spawn position [m]                 (0.0, 0.0, 0.85)
  base_roll/pitch/yaw : float spawn orientation [rad]        (0, 0, 0)
  srdf_file       : str   SRDF path ("" -> arm_moveit_config)
  arm_controller  : str   JTC for the arm                    (left_arm_controller)
  arm_state       : str   <group_state> for the arm          (home)
  hand_controller : str   JTC for the hand ("" -> skip)      (left_hand_controller)
  hand_state      : str   <group_state> for the hand         (open)
  duration        : float joint move time [s]                (2.0)

NOTE: base_x/y/z default to spawn_arm.launch.py's defaults (0, 0, 0.85). If you
spawn the robot elsewhere, override them. set_pose teleports without zeroing
velocity, so trigger when the robot is roughly at rest.
"""

import math
import os
import subprocess
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def rpy_to_quat(roll, pitch, yaw):
    """ZYX intrinsic Euler (roll-pitch-yaw) -> quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


def parse_srdf_state(path, state_name):
    """Return ({joint: value}, None) for the named <group_state>, or (None, err)."""
    try:
        root = ET.parse(path).getroot()
    except (ET.ParseError, OSError) as exc:
        return None, str(exc)
    for gs in root.findall("group_state"):
        if gs.get("name") == state_name:
            joints = {}
            for j in gs.findall("joint"):
                name, val = j.get("name"), j.get("value")
                if name is not None and val is not None:
                    joints[name] = float(val)
            if joints:
                return joints, None
            return None, f"group_state '{state_name}' has no joints"
    return None, f"group_state '{state_name}' not found"


def default_srdf_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory("arm_moveit_config"),
            "config", "arm_description.srdf",
        )
    except Exception:  # noqa: BLE001 - package may be missing
        return ""


class ResetRobot(Node):
    def __init__(self):
        super().__init__("reset_robot")

        # --- base teleport ---
        self.declare_parameter("reset_base", True)
        self.declare_parameter("model", "arm")
        self.declare_parameter("world", "")
        self.declare_parameter("base_x", 0.0)
        self.declare_parameter("base_y", 0.0)
        self.declare_parameter("base_z", 0.85)
        self.declare_parameter("base_roll", 0.0)
        self.declare_parameter("base_pitch", 0.0)
        self.declare_parameter("base_yaw", 0.0)
        # --- joint reset ---
        self.declare_parameter("srdf_file", "")
        self.declare_parameter("topic", "/reset_robot")
        self.declare_parameter("arm_controller", "left_arm_controller")
        self.declare_parameter("arm_state", "home")
        self.declare_parameter("hand_controller", "left_hand_controller")
        self.declare_parameter("hand_state", "open")
        self.declare_parameter("duration", 2.0)

        self.reset_base = bool(self.get_parameter("reset_base").value)
        self.model = self.get_parameter("model").value
        self._world = self.get_parameter("world").value or None
        topic = self.get_parameter("topic").value
        self.duration = float(self.get_parameter("duration").value)
        srdf = self.get_parameter("srdf_file").value or default_srdf_path()

        # Resolve each controller's target pose from the SRDF named state.
        self.targets = []  # (controller_topic, joint_names, positions, label)
        groups = [
            (self.get_parameter("arm_controller").value,
             self.get_parameter("arm_state").value),
            (self.get_parameter("hand_controller").value,
             self.get_parameter("hand_state").value),
        ]
        for controller, state in groups:
            if not controller or not state:
                continue
            joints, err = parse_srdf_state(srdf, state) if srdf else (None, "no SRDF")
            if err:
                self.get_logger().warn(f"{controller}: cannot load '{state}': {err}")
                continue
            names = list(joints.keys())
            self.targets.append((
                f"/{controller}/joint_trajectory", names,
                [joints[n] for n in names], f"{controller}->{state}",
            ))

        self.pubs = {
            t[0]: self.create_publisher(JointTrajectory, t[0], 10)
            for t in self.targets
        }
        self.create_subscription(Empty, topic, self.on_reset, 10)

        base = (
            "base->[{:.4g}, {:.4g}, {:.4g}]".format(
                self.get_parameter("base_x").value,
                self.get_parameter("base_y").value,
                self.get_parameter("base_z").value)
            if self.reset_base else "base off"
        )
        joints = ", ".join(t[3] for t in self.targets) or "no joint targets"
        self.get_logger().info(
            f"Publish std_msgs/Empty on '{topic}' to reset robot "
            f"(model='{self.model}', {base}; {joints})."
        )

    # ---- base teleport via Gazebo set_pose ----
    def resolve_world(self):
        if self._world:
            return self._world
        try:
            out = subprocess.run(
                ["gz", "service", "-l"], capture_output=True, text=True, timeout=5
            )
        except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
            self.get_logger().error(f"could not list gz services: {exc}")
            return None
        for line in out.stdout.splitlines():
            line = line.strip()
            if line.startswith("/world/") and line.endswith("/set_pose"):
                self._world = line[len("/world/"):-len("/set_pose")]
                return self._world
        self.get_logger().error("no /world/*/set_pose service (is the sim running?)")
        return None

    def teleport_base(self):
        world = self.resolve_world()
        if world is None:
            return
        px = self.get_parameter("base_x").value
        py = self.get_parameter("base_y").value
        pz = self.get_parameter("base_z").value
        qx, qy, qz, qw = rpy_to_quat(
            self.get_parameter("base_roll").value,
            self.get_parameter("base_pitch").value,
            self.get_parameter("base_yaw").value,
        )
        req = (
            f'name: "{self.model}", '
            f"position: {{x: {px}, y: {py}, z: {pz}}}, "
            f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}"
        )
        cmd = [
            "gz", "service", "-s", f"/world/{world}/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req", req,
        ]
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
            self.get_logger().error(f"set_pose call failed: {exc}")
            return
        if out.returncode == 0 and "true" in out.stdout.lower():
            self.get_logger().info(
                f"Stood '{self.model}' up -> [{px:.4g}, {py:.4g}, {pz:.4g}]."
            )
        else:
            self.get_logger().error(
                f"set_pose returned '{out.stdout.strip()}' "
                f"err '{out.stderr.strip()}' (is the sim running?)."
            )

    # ---- joint reset via trajectory controllers ----
    def reset_joints(self):
        if not self.targets:
            self.get_logger().error("no joint targets; check srdf_file / state names.")
            return
        sec = int(self.duration)
        nanosec = int((self.duration - sec) * 1e9)
        for ctrl_topic, names, positions, label in self.targets:
            traj = JointTrajectory()
            traj.joint_names = names
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            traj.points = [point]
            self.pubs[ctrl_topic].publish(traj)
            self.get_logger().info(f"Reset {label} ({self.duration:g}s).")

    def on_reset(self, _msg):
        if self.reset_base:
            self.teleport_base()   # A. stand it back up first
        self.reset_joints()        # B. then drive joints home


def main():
    rclpy.init()
    node = ResetRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
