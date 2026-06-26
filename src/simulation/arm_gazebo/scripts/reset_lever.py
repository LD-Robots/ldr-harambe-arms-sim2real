#!/usr/bin/env python3
"""
Reset the lever (pickable cylinder) back to its start pose in Gazebo.

During MTC pick-and-place testing the robot knocks the lever over (or carries it
away), and you don't want to restart the whole sim to try again. Publish an empty
message to /reset_lever and this node teleports the lever back to its home pose via
the Gazebo Harmonic UserCommands `set_pose` service.

World-agnostic
--------------
The lever's home pose AND the world name differ per world (lab-ldr: world "lab",
lever at [-0.30, 0.35, 0.5]; lab-mtc: world "whole_arm_manipulation_workspace",
lever at [0.45, 0.35, 1.025]). Rather than hardcode, point the node at the world
SDF via `world_file` and it reads both the world name and the lever's <pose>
straight from the file — so it just works for whichever world Gazebo launched.

Resolution order (each item overrides the next):
  world name : `world` param  ->  parsed from `world_file`  ->  auto-detect via gz
  home pose  : explicit x/y/z params  ->  parsed from `world_file`

Usage
-----
    # Auto from the launched world file (how headless.launch.py wires it):
    ros2 run arm_gazebo reset_lever --ros-args -p world_file:=/path/to/lab-mtc.sdf

    # Trigger a reset:
    ros2 topic pub --once /reset_lever std_msgs/msg/Empty {}

The set_pose service is provided by the gz-sim-user-commands-system plugin, which
both lab-mtc.sdf and lab-ldr.sdf load.
"""

import math
import subprocess
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


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


def parse_world_file(path, model_name):
    """Return (world_name, [x, y, z, r, p, y]) for `model_name`, or (None, None)."""
    try:
        root = ET.parse(path).getroot()
    except (ET.ParseError, OSError) as exc:
        return None, None, str(exc)
    world = root.find("world")
    if world is None:
        return None, None, "no <world> element"
    world_name = world.get("name")
    for model in world.findall("model"):
        if model.get("name") == model_name:
            pose_el = model.find("pose")  # the model's OWN pose (direct child)
            if pose_el is not None and pose_el.text:
                vals = [float(v) for v in pose_el.text.split()]
                if len(vals) >= 6:
                    return world_name, vals[:6], None
            return world_name, None, f"model '{model_name}' has no <pose>"
    return world_name, None, f"model '{model_name}' not found"


class ResetLever(Node):
    def __init__(self):
        super().__init__("reset_lever")

        self.declare_parameter("world_file", "")  # SDF to read world name + home pose
        self.declare_parameter("world", "")        # explicit world-name override
        self.declare_parameter("model", "lever")
        self.declare_parameter("topic", "/reset_lever")
        nan = float("nan")
        self.declare_parameter("x", nan)           # explicit pose override
        self.declare_parameter("y", nan)
        self.declare_parameter("z", nan)
        self.declare_parameter("roll", nan)
        self.declare_parameter("pitch", nan)
        self.declare_parameter("yaw", nan)

        self.model = self.get_parameter("model").value
        topic = self.get_parameter("topic").value
        self._world = self.get_parameter("world").value or None
        self._home = None  # [x, y, z, r, p, y]

        # 1) From the world SDF, if given.
        world_file = self.get_parameter("world_file").value
        if world_file:
            wname, pose, err = parse_world_file(world_file, self.model)
            if err:
                self.get_logger().warn(f"world_file '{world_file}': {err}")
            if self._world is None and wname:
                self._world = wname
            if pose is not None:
                self._home = pose

        # 2) Explicit pose params win if x/y/z were all set.
        ex, ey, ez = (self.get_parameter(k).value for k in ("x", "y", "z"))
        if all(math.isfinite(v) for v in (ex, ey, ez)):
            rpy = [self.get_parameter(k).value for k in ("roll", "pitch", "yaw")]
            rpy = [v if math.isfinite(v) else 0.0 for v in rpy]
            self._home = [ex, ey, ez, *rpy]

        self.create_subscription(Empty, topic, self.on_reset, 10)
        home = (
            "[{:.4g}, {:.4g}, {:.4g}] rpy [{:.4g}, {:.4g}, {:.4g}]".format(*self._home)
            if self._home else "<unset — pass world_file or x/y/z>"
        )
        self.get_logger().info(
            f"Publish std_msgs/Empty on '{topic}' to reset '{self.model}' "
            f"(world={self._world or 'auto'}, home={home})."
        )

    def resolve_world(self):
        """World name: cached/explicit, else discover from the running sim."""
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
                self.get_logger().info(f"Auto-detected world '{self._world}'.")
                return self._world
        self.get_logger().error("no /world/*/set_pose service (is the sim running?)")
        return None

    def on_reset(self, _msg):
        if self._home is None:
            self.get_logger().error("no home pose known; set world_file or x/y/z.")
            return
        world = self.resolve_world()
        if world is None:
            return

        px, py, pz, roll, pitch, yaw = self._home
        qx, qy, qz, qw = rpy_to_quat(roll, pitch, yaw)
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
                f"Reset '{self.model}' -> [{px:.4g}, {py:.4g}, {pz:.4g}]."
            )
        else:
            self.get_logger().error(
                f"set_pose returned '{out.stdout.strip()}' "
                f"err '{out.stderr.strip()}' (is the sim running?)."
            )


def main():
    rclpy.init()
    node = ResetLever()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
