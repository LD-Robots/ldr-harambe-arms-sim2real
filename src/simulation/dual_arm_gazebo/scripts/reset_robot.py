#!/usr/bin/env python3
"""
Reset the dual-arm robot back to its upright home state in Gazebo.

Publish std_msgs/Empty on /reset_robot to:
  A. Teleport the base back to its spawn pose (Gazebo UserCommands set_pose).
  B. Command the trajectory controllers back to their SRDF named states.

Started as a copy of arm_gazebo/scripts/reset_robot.py. Two things differ, both
forced by the dual-arm setup:

  * The SRDF has several states per name -- `home` exists for left_arm, right_arm
    and both_arms -- so a state is selected by (group, name), not by name alone.
    The single-arm version takes the first match, which here is the wrong one.

  * waist_yaw_joint_X8 is the first joint of both arm chains, so it belongs to
    neither arm controller and has a controller of its own. A group state therefore
    spans more than one controller: left_arm/home lists seven joints, six owned by
    left_arm_controller and one by waist_controller. Each controller is asked for
    its own `joints` parameter and sent only the intersection. Publishing a joint a
    controller does not own makes JointTrajectoryController reject the whole
    trajectory.

Parameters
----------
    reset_base   : bool   teleport the base as well              (True)
    model        : str    Gazebo entity name                     (dual_arm)
    world        : str    world name ("" -> discover via gz)     ("")
    base_x/y/z   : float  spawn pose                             (0, 0, 0.85)
    base_roll/pitch/yaw : float                                  (0, 0, 0)
    srdf_file    : str    SRDF path ("" -> dual_arm_moveit_config)
    topic        : str    trigger topic                          (/reset_robot)
    controllers  : str[]  controllers to command
    groups       : str[]  SRDF group per controller, parallel to `controllers`
    states       : str[]  SRDF state per controller, parallel to `controllers`
    duration     : float  seconds for the motion                 (2.0)

Usage
-----
    ros2 topic pub --once /reset_robot std_msgs/msg/Empty {}
"""

import math
import os
import subprocess
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def parse_srdf_state(path, group, state_name):
    """Return ({joint: value}, None) for <group_state name=.. group=..>, or (None, err)."""
    try:
        root = ET.parse(path).getroot()
    except (ET.ParseError, OSError) as exc:
        return None, str(exc)
    for gs in root.findall("group_state"):
        if gs.get("name") != state_name or gs.get("group") != group:
            continue
        joints = {j.get("name"): float(j.get("value"))
                  for j in gs.findall("joint")
                  if j.get("name") is not None and j.get("value") is not None}
        if joints:
            return joints, None
        return None, f"group_state '{group}/{state_name}' has no joints"
    return None, f"group_state '{group}/{state_name}' not found"


def default_srdf_path():
    try:
        return os.path.join(get_package_share_directory("dual_arm_moveit_config"),
                            "config", "dual_arm_description.srdf")
    except Exception:  # noqa: BLE001 - package not built yet
        return ""


class ResetRobot(Node):
    def __init__(self):
        super().__init__("reset_robot")

        self.declare_parameter("reset_base", True)
        self.declare_parameter("model", "dual_arm")
        self.declare_parameter("world", "")
        self.declare_parameter("base_x", 0.0)
        self.declare_parameter("base_y", 0.0)
        self.declare_parameter("base_z", 0.85)
        self.declare_parameter("base_roll", 0.0)
        self.declare_parameter("base_pitch", 0.0)
        self.declare_parameter("base_yaw", 0.0)
        self.declare_parameter("duration", 2.0)
        self.declare_parameter("srdf_file", "")
        self.declare_parameter("topic", "/reset_robot")
        self.declare_parameter("controllers", [
            "waist_controller", "left_arm_controller", "right_arm_controller",
            "left_hand_controller", "right_hand_controller"])
        self.declare_parameter("groups", [
            "left_arm", "left_arm", "right_arm", "left_hand", "right_hand"])
        self.declare_parameter("states", [
            "home", "home", "home", "open", "open"])

        self.reset_base = bool(self.get_parameter("reset_base").value)
        self.model = self.get_parameter("model").value
        self._world = self.get_parameter("world").value or None
        self.duration = float(self.get_parameter("duration").value)
        topic = self.get_parameter("topic").value
        srdf = self.get_parameter("srdf_file").value or default_srdf_path()

        controllers = list(self.get_parameter("controllers").value)
        groups = list(self.get_parameter("groups").value)
        states = list(self.get_parameter("states").value)
        if not (len(controllers) == len(groups) == len(states)):
            self.get_logger().error(
                f"controllers({len(controllers)}), groups({len(groups)}) and "
                f"states({len(states)}) must have the same length")
            controllers = groups = states = []

        # {controller: {joint: value}} straight from the SRDF, before filtering.
        self.wanted = {}
        for controller, group, state in zip(controllers, groups, states):
            joints, err = parse_srdf_state(srdf, group, state) if srdf else (None, "no SRDF")
            if err:
                self.get_logger().warn(f"{controller}: cannot load '{group}/{state}': {err}")
                continue
            self.wanted[controller] = joints

        self.pubs = {c: self.create_publisher(JointTrajectory, f"/{c}/joint_trajectory", 10)
                     for c in self.wanted}
        # Controllers are not up when this node starts, so their joint lists are
        # fetched on the first reset and cached from then on.
        self.owned = {}
        self.param_clients = {c: self.create_client(GetParameters, f"/{c}/get_parameters")
                              for c in self.wanted}

        self.create_subscription(Empty, topic, self.on_reset, 10)
        base = ("base->[{:.4g}, {:.4g}, {:.4g}]".format(
                    self.get_parameter("base_x").value,
                    self.get_parameter("base_y").value,
                    self.get_parameter("base_z").value)
                if self.reset_base else "base off")
        self.get_logger().info(
            f"Publish std_msgs/Empty on '{topic}' to reset robot "
            f"(model='{self.model}', {base}; controllers: {', '.join(self.wanted) or 'none'}).")

    # ---- base teleport via Gazebo set_pose ----
    def resolve_world(self):
        if self._world:
            return self._world
        try:
            out = subprocess.run(["gz", "service", "-l"],
                                 capture_output=True, text=True, timeout=5)
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
        qx, qy, qz, qw = rpy_to_quat(self.get_parameter("base_roll").value,
                                     self.get_parameter("base_pitch").value,
                                     self.get_parameter("base_yaw").value)
        req = (f'name: "{self.model}", '
               f"position: {{x: {px}, y: {py}, z: {pz}}}, "
               f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}")
        cmd = ["gz", "service", "-s", f"/world/{world}/set_pose",
               "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
               "--timeout", "3000", "--req", req]
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
            self.get_logger().error(f"set_pose call failed: {exc}")
            return
        if out.returncode == 0 and "true" in out.stdout.lower():
            self.get_logger().info(
                f"Stood '{self.model}' up -> [{px:.4g}, {py:.4g}, {pz:.4g}].")
        else:
            self.get_logger().error(
                f"set_pose returned '{out.stdout.strip()}' "
                f"err '{out.stderr.strip()}' (is the sim running?).")

    # ---- joint reset via trajectory controllers ----
    def publish_for(self, controller):
        """Send the joints of this controller's target state that it actually owns."""
        owned = self.owned.get(controller)
        wanted = self.wanted[controller]
        names = [j for j in wanted if j in owned] if owned else list(wanted)
        if not names:
            self.get_logger().warn(
                f"{controller}: none of {list(wanted)} are owned by it; nothing sent")
            return
        skipped = [j for j in wanted if j not in names]
        sec = int(self.duration)
        traj = JointTrajectory()
        traj.joint_names = names
        traj.points = [JointTrajectoryPoint(
            positions=[wanted[n] for n in names],
            time_from_start=Duration(sec=sec, nanosec=int((self.duration - sec) * 1e9)))]
        self.pubs[controller].publish(traj)
        note = f" (skipped {', '.join(skipped)}, owned elsewhere)" if skipped else ""
        self.get_logger().info(
            f"Reset {controller}: {len(names)} joint(s), {self.duration:g}s{note}.")

    def on_reset(self, _msg):
        if self.reset_base:
            self.teleport_base()
        if not self.wanted:
            self.get_logger().error("no joint targets; check srdf_file / group / state names.")
            return
        for controller in self.wanted:
            if controller in self.owned:
                self.publish_for(controller)
                continue
            client = self.param_clients[controller]
            if not client.service_is_ready():
                self.get_logger().warn(
                    f"{controller}: parameters not available, sending the state unfiltered")
                self.publish_for(controller)
                continue
            request = GetParameters.Request(names=["joints"])
            client.call_async(request).add_done_callback(
                lambda future, c=controller: self._on_joints(future, c))

    def _on_joints(self, future, controller):
        try:
            values = future.result().values
            self.owned[controller] = set(values[0].string_array_value) if values else set()
        except Exception as exc:  # noqa: BLE001 - fall back to sending everything
            self.get_logger().warn(f"{controller}: could not read joints: {exc}")
            self.owned[controller] = set()
        self.publish_for(controller)


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
