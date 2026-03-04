#!/usr/bin/env python3
"""
Gravity compensation node for kinesthetic teaching.

Computes gravity torques from the URDF using PyKDL and publishes them
to the effort controller, allowing the arm to "float" while the user
physically guides it.

Requires:
  - PyKDL (python3-pykdl)
  - urdf_parser_py
  - /robot_description topic published by robot_state_publisher
  - effort controller active (e.g. left_arm_effort_controller)

Usage:
    ros2 run arm_real_bringup gravity_comp_node.py

Torque conversion (N·m → effort units):
    effort * 50 = raw PDO = per-mille of rated current (10A)
    So: effort 1.0 = 0.5A through the motor.

    Module Torque Constants (Kt at output, gear ratio already included):
      X6: Kt=2.1 Nm/A → 1.0 effort = 0.5A * 2.1 = 1.05 Nm
      X4: Kt=1.9 Nm/A → 1.0 effort = 0.5A * 1.9 = 0.95 Nm

    Default torque_scale converts N·m → effort:
      X6 joints: 1/1.05 ≈ 0.952
      X4 joints: 1/0.95 ≈ 1.053

Parameters:
    torque_scale: Per-joint N·m → effort conversion (default: computed from Kt)
    max_torque: Per-joint effort clamp (default: 10.0/8.0 = ~10.5/7.6 Nm)
    publish_rate: Hz (default: 100)
    controller_topic: Effort controller command topic
    root_link: KDL chain root (default: urdf_simplified_torso)
    tip_link: KDL chain tip (default: urdf_l_wrist_assembly)
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

try:
    import PyKDL
except ImportError:
    # ros2 launch may not include /usr/lib/python3/dist-packages in PYTHONPATH
    import sys
    sys.path.insert(0, "/usr/lib/python3/dist-packages")
    import PyKDL

try:
    from urdf_parser_py.urdf import URDF
    KDL_AVAILABLE = True
except ImportError as e:
    KDL_AVAILABLE = False
    KDL_IMPORT_ERROR = str(e)

JOINT_NAMES = [
    "left_shoulder_pitch_joint_X6",
    "left_shoulder_roll_joint_X6",
    "left_shoulder_yaw_joint_X4",
    "left_elbow_pitch_joint_X6",
    "left_wrist_yaw_joint_X4",
    "left_wrist_roll_joint_X4",
]

# Short names for logging
JOINT_SHORT = ["sh_pitch", "sh_roll", "sh_yaw", "el_pitch", "wr_yaw", "wr_roll"]

# Motor specs — "Module Torque Constant" is at the OUTPUT (gear ratio already included)
#   Proof: X6 rated_torque=20Nm / rated_current=9.5A = 2.1 Nm/A ✓
#   effort 1.0 = 0.5A (raw PDO 50 = 5% of 10A rated)
MOTOR_KT = {"X6": 2.1, "X4": 1.9}  # Nm/A at output (after gearbox)

# Joint motor type mapping (order matches JOINT_NAMES)
JOINT_MOTOR_TYPE = ["X6", "X6", "X4", "X6", "X4", "X4"]

# Nm per 1.0 effort for each joint (Kt is already at output, no gear ratio needed)
#   X6: 0.5A * 2.1 = 1.05 Nm/effort
#   X4: 0.5A * 1.9 = 0.95 Nm/effort
NM_PER_EFFORT = [0.5 * MOTOR_KT[mt] for mt in JOINT_MOTOR_TYPE]

# Default torque_scale: N·m → effort (inverse of NM_PER_EFFORT)
DEFAULT_TORQUE_SCALE = [1.0 / npe for npe in NM_PER_EFFORT]

# Default per-joint safety clamp in effort units
# shoulder_pitch needs up to ~13 effort at horizontal extension
# X6: effort 1.0 = 1.05 Nm, X4: effort 1.0 = 0.95 Nm
DEFAULT_MAX_TORQUE = [15.0, 10.0, 8.0, 10.0, 8.0, 8.0]


def urdf_pose_to_kdl_frame(pose):
    """Convert a URDF pose (xyz + rpy) to a KDL Frame."""
    if pose is None:
        return PyKDL.Frame.Identity()
    x, y, z = pose.xyz if pose.xyz else (0, 0, 0)
    r, p, yaw = pose.rpy if pose.rpy else (0, 0, 0)
    return PyKDL.Frame(
        PyKDL.Rotation.RPY(r, p, yaw),
        PyKDL.Vector(x, y, z),
    )


def urdf_inertial_to_kdl(inertial):
    """Convert URDF inertial to KDL RigidBodyInertia."""
    if inertial is None:
        return PyKDL.RigidBodyInertia()

    origin = urdf_pose_to_kdl_frame(inertial.origin)
    mass = inertial.mass or 0.0
    cog = origin.p  # Center of gravity in link frame

    ixx = inertial.inertia.ixx or 0.0
    ixy = inertial.inertia.ixy or 0.0
    ixz = inertial.inertia.ixz or 0.0
    iyy = inertial.inertia.iyy or 0.0
    iyz = inertial.inertia.iyz or 0.0
    izz = inertial.inertia.izz or 0.0

    return PyKDL.RigidBodyInertia(
        mass, cog,
        PyKDL.RotationalInertia(ixx, iyy, izz, ixy, ixz, iyz),
    )


def urdf_joint_to_kdl_joint(joint):
    """Convert a URDF joint to a KDL Joint."""
    origin = urdf_pose_to_kdl_frame(joint.origin)

    if joint.type == "fixed":
        return PyKDL.Joint(joint.name, PyKDL.Joint.Fixed)

    axis = PyKDL.Vector(*(joint.axis or (1, 0, 0)))

    if joint.type in ("revolute", "continuous"):
        return PyKDL.Joint(
            joint.name, origin.p, origin.M * axis, PyKDL.Joint.RotAxis
        )
    elif joint.type == "prismatic":
        return PyKDL.Joint(
            joint.name, origin.p, origin.M * axis, PyKDL.Joint.TransAxis
        )
    else:
        return PyKDL.Joint(joint.name, PyKDL.Joint.Fixed)


def build_kdl_chain_from_urdf(urdf_string, root_link, tip_link):
    """Build a KDL chain from URDF string between root_link and tip_link.

    This replaces kdl_parser_py which is not available on Jazzy.
    """
    # Encode to bytes to handle XML encoding declarations (lxml requirement)
    if isinstance(urdf_string, str):
        urdf_bytes = urdf_string.encode("utf-8")
    else:
        urdf_bytes = urdf_string
    robot = URDF.from_xml_string(urdf_bytes)

    # Build parent map: child_link -> (joint, parent_link)
    parent_map = {}
    for joint in robot.joints:
        parent_map[joint.child] = (joint, joint.parent)

    # Find chain from tip to root
    chain_joints = []
    current = tip_link
    while current != root_link:
        if current not in parent_map:
            raise ValueError(
                f"Cannot find path from '{tip_link}' to '{root_link}'. "
                f"Link '{current}' has no parent joint."
            )
        joint, parent = parent_map[current]
        chain_joints.append((joint, current))
        current = parent

    # Reverse to get root→tip order
    chain_joints.reverse()

    # Build link map for inertial data
    link_map = {link.name: link for link in robot.links}

    # Create KDL chain
    chain = PyKDL.Chain()
    for joint, child_link_name in chain_joints:
        kdl_joint = urdf_joint_to_kdl_joint(joint)
        kdl_frame = urdf_pose_to_kdl_frame(joint.origin)

        child_link = link_map.get(child_link_name)
        kdl_inertia = urdf_inertial_to_kdl(
            child_link.inertial if child_link else None
        )

        segment = PyKDL.Segment(
            child_link_name, kdl_joint, kdl_frame, kdl_inertia
        )
        chain.addSegment(segment)

    return chain


class GravityCompNode(Node):

    def __init__(self):
        super().__init__("gravity_comp_node")

        if not KDL_AVAILABLE:
            self.get_logger().fatal(
                f"PyKDL or urdf_parser_py not available: {KDL_IMPORT_ERROR}\n"
                "Install with: sudo apt install python3-pykdl python3-urdf-parser"
            )
            raise RuntimeError("Missing KDL dependencies")

        # Parameters
        self._torque_scale = list(
            self.declare_parameter("torque_scale", DEFAULT_TORQUE_SCALE)
            .get_parameter_value().double_array_value
        )
        self._max_torque = list(
            self.declare_parameter("max_torque", DEFAULT_MAX_TORQUE)
            .get_parameter_value().double_array_value
        )
        self._publish_rate = (
            self.declare_parameter("publish_rate", 100.0)
            .get_parameter_value().double_value
        )
        self._controller_topic = (
            self.declare_parameter(
                "controller_topic",
                "/left_arm_effort_controller/commands"
            ).get_parameter_value().string_value
        )
        self._root_link = (
            self.declare_parameter("root_link", "urdf_simplified_torso")
            .get_parameter_value().string_value
        )
        self._tip_link = (
            self.declare_parameter("tip_link", "urdf_l_wrist_assembly")
            .get_parameter_value().string_value
        )
        self._log_interval = (
            self.declare_parameter("log_interval", 2.0)
            .get_parameter_value().double_value
        )

        # State
        self._joint_index = {name: i for i, name in enumerate(JOINT_NAMES)}
        self._lock = threading.Lock()
        self._current_positions = None
        self._kdl_chain = None
        self._gravity_solver = None
        self._enabled = False
        self._last_log_time = 0.0
        self._clamp_count = [0] * len(JOINT_NAMES)
        self._publish_count = 0

        # Subscribe to /robot_description to get URDF
        self._urdf_sub = self.create_subscription(
            String, "/robot_description",
            self._urdf_callback, QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            )
        )
        self.get_logger().info("Waiting for /robot_description...")

        # Publisher
        self._pub = self.create_publisher(
            Float64MultiArray, self._controller_topic, 10
        )

        # Subscriber
        self._sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

        # Timer for publishing gravity torques
        period = 1.0 / self._publish_rate
        self._timer = self.create_timer(period, self._publish_gravity_torques)

    def _urdf_callback(self, msg):
        """Process URDF string and build KDL chain."""
        if self._kdl_chain is not None:
            return  # Already initialized

        urdf_string = msg.data
        self.get_logger().info("Received URDF, building KDL chain...")

        try:
            chain = build_kdl_chain_from_urdf(
                urdf_string, self._root_link, self._tip_link
            )
        except Exception as e:
            self.get_logger().error(f"Failed to build KDL chain: {e}")
            return

        if chain.getNrOfJoints() == 0:
            self.get_logger().error(
                f"KDL chain from '{self._root_link}' to '{self._tip_link}' "
                f"has 0 joints! Check link names."
            )
            return

        self.get_logger().info(
            f"KDL chain built: {chain.getNrOfJoints()} joints, "
            f"{chain.getNrOfSegments()} segments"
        )

        # Gravity vector (pointing down in world frame)
        gravity = PyKDL.Vector(0, 0, -9.81)
        self._gravity_solver = PyKDL.ChainDynParam(chain, gravity)
        self._kdl_chain = chain
        self._enabled = True

        max_str = ", ".join(f"{v:.2f}" for v in self._max_torque)
        scale_str = ", ".join(f"{v:.5f}" for v in self._torque_scale)
        n = len(JOINT_NAMES)
        max_nm = [self._max_torque[i] * NM_PER_EFFORT[i] for i in range(n)]
        max_nm_str = ", ".join(f"{v:.1f}" for v in max_nm)
        self.get_logger().info(
            f"Gravity compensation ACTIVE.\n"
            f"  Chain: {self._root_link} -> {self._tip_link}\n"
            f"  Joints: {chain.getNrOfJoints()}\n"
            f"  Rate: {self._publish_rate} Hz\n"
            f"  Scale (Nm->effort): [{scale_str}]\n"
            f"  Max effort: [{max_str}]\n"
            f"  Max torque: [{max_nm_str}] Nm\n"
            f"  Topic: {self._controller_topic}"
        )

        # Unsubscribe from URDF topic
        self.destroy_subscription(self._urdf_sub)
        self._urdf_sub = None

    def _joint_state_cb(self, msg):
        """Extract current joint positions."""
        positions = [None] * len(JOINT_NAMES)
        for name, pos in zip(msg.name, msg.position):
            if name in self._joint_index:
                positions[self._joint_index[name]] = pos

        if any(v is None for v in positions):
            return

        with self._lock:
            self._current_positions = [float(p) for p in positions]

    def _publish_gravity_torques(self):
        """Compute and publish gravity compensation torques."""
        if not self._enabled:
            return

        with self._lock:
            if self._current_positions is None:
                return
            positions = list(self._current_positions)

        n_joints = len(JOINT_NAMES)

        # Build KDL joint array
        q = PyKDL.JntArray(n_joints)
        for i in range(n_joints):
            q[i] = positions[i]

        # Compute gravity torques (N·m at joint output)
        gravity_torques = PyKDL.JntArray(n_joints)
        result = self._gravity_solver.JntToGravity(q, gravity_torques)
        if result != 0:
            self.get_logger().warn(f"JntToGravity failed with code: {result}")
            return

        # Apply scaling (N·m -> effort units) and per-joint safety clamp
        msg = Float64MultiArray()
        raw_nm = []
        sent_effort = []
        for i in range(n_joints):
            nm = gravity_torques[i]
            effort = nm * self._torque_scale[i]
            limit = self._max_torque[i] if i < len(self._max_torque) else 0.5
            clamped = max(-limit, min(limit, effort))
            if abs(clamped) < abs(effort):
                self._clamp_count[i] += 1
            msg.data.append(clamped)
            raw_nm.append(nm)
            sent_effort.append(clamped)

        self._pub.publish(msg)
        self._publish_count += 1

        # Periodic logging for tuning
        now = time.monotonic()
        if now - self._last_log_time >= self._log_interval:
            self._last_log_time = now
            lines = ["Gravity comp torques:"]
            lines.append(
                f"  {'Joint':<12} {'KDL(Nm)':>9} {'Sent(Nm)':>9} "
                f"{'Effort':>8} {'Max':>6} {'Clamp':>6}"
            )
            for i in range(n_joints):
                sent_nm = sent_effort[i] * NM_PER_EFFORT[i]
                clamp_flag = " *" if self._clamp_count[i] > 0 else ""
                lines.append(
                    f"  {JOINT_SHORT[i]:<12} "
                    f"{raw_nm[i]:>+9.3f} "
                    f"{sent_nm:>+9.3f} "
                    f"{sent_effort[i]:>+8.4f} "
                    f"{self._max_torque[i]:>6.2f} "
                    f"{self._clamp_count[i]:>5d}{clamp_flag}"
                )
            self._clamp_count = [0] * n_joints
            self.get_logger().info("\n".join(lines))

    def _ramp_effort(self, target, current, max_step=0.1):
        """Smooth ramp to avoid sudden torque jumps."""
        delta = target - current
        if abs(delta) <= max_step:
            return target
        return current + max_step * (1.0 if delta > 0 else -1.0)


def main():
    rclpy.init()

    try:
        node = GravityCompNode()
    except RuntimeError:
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Gravity compensation stopped.")
    finally:
        # Publish zero torques on shutdown
        try:
            msg = Float64MultiArray()
            msg.data = [0.0] * len(JOINT_NAMES)
            node._pub.publish(msg)
            node.get_logger().info("Zero torques sent.")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
