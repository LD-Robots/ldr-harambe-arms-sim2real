#!/usr/bin/env python3
"""
Kinematic chain debug tool for the 25-joint humanoid.

Dumps, per controlled joint, the runtime quantities that matter for gravity
compensation and chain-coupling intuition:

  • joint axis in world frame (and its horizontal component — how much
    gravity can couple to that joint at this pose)
  • joint pivot position in world frame
  • child link's mass + world-frame COM
  • per-joint gravity torque from Pinocchio's RNEA — the actuator torque
    the controller will actually command
  • sensitivity matrix ∂tau_g_j / ∂q_k — how much each joint's gravity
    load shifts when each upstream joint moves (static mode only;
    a numeric central-difference column-wise around the current pose)

Two modes:
  static    one-shot snapshot from a pose YAML (default: URDF reference /
            all zeros). No ROS needed. Right for offline analysis, URDF
            symmetry checks, sign-convention debugging.
  live      subscribes to /joint_states_raw, refreshes the snapshot at
            5 Hz. Right for bench work: back-drive a joint and watch the
            world-frame quantities and gravity loads change.

Usage:
    chain_debug.py static
    chain_debug.py static --pose src/bringup/robot_bringup/config/relaxed_pose.yaml
    chain_debug.py live
"""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

# Pinocchio's C extension on Jazzy was compiled against numpy 1.x. If the
# user has numpy 2.x installed in ~/.local, the import path resolves to that
# wheel and pinocchio segfaults. Strip user-site from sys.path before any
# numpy / pinocchio import.
sys.path = [
    p for p in sys.path
    if "/.local/lib" not in p and "isaac-sim" not in p
]

import numpy as np   # noqa: E402
import yaml          # noqa: E402

try:
    import pinocchio as pin
except ImportError as e:
    sys.stderr.write(
        f"pinocchio not importable: {e}\n"
        "Did you `sudo apt install ros-jazzy-pinocchio` and source the workspace?\n"
    )
    raise


# Joint roster used by the PVT controller — matches controllers_pvt.yaml.
JOINT_ROSTER = [
    "left_shoulder_pitch_joint_X6",
    "left_shoulder_roll_joint_X6",
    "left_shoulder_yaw_joint_X4",
    "left_elbow_pitch_joint_X6",
    "left_wrist_yaw_joint_X4",
    "left_wrist_roll_joint_X4",
    "right_shoulder_pitch_joint_X6",
    "right_shoulder_roll_joint_X6",
    "right_shoulder_yaw_joint_X4",
    "right_elbow_pitch_joint_X6",
    "right_wrist_yaw_joint_X4",
    "right_wrist_roll_joint_X4",
    "waist_yaw_joint_X8",
    "left_hip_pitch_joint_X8",
    "left_hip_roll_joint_X8",
    "left_hip_yaw_joint_X8",
    "left_knee_joint_X8",
    "left_ankle_pitch_joint_X4",
    "left_ankle_roll_joint_X4",
    "right_hip_pitch_joint_X8",
    "right_hip_roll_joint_X8",
    "right_hip_yaw_joint_X8",
    "right_knee_joint_X8",
    "right_ankle_roll_joint_X4",
    "right_ankle_pitch_joint_X4",
]

REPO_SRC = Path(__file__).resolve().parents[3]
DEFAULT_XACRO = (
    REPO_SRC
    / "robot_description/full_robot_description/urdf/full_robot.urdf.xacro"
)


# ── URDF + Pinocchio loading ────────────────────────────────────────────────

def load_urdf_xml(xacro_path=None, urdf_path=None):
    """Return the URDF XML string (running xacro if needed)."""
    if urdf_path is not None:
        return Path(urdf_path).read_text()
    xacro_bin = shutil.which("xacro")
    if xacro_bin is None:
        sys.stderr.write(
            "xacro not on PATH. Source /opt/ros/jazzy/setup.bash first.\n"
        )
        sys.exit(1)
    return subprocess.run(
        [xacro_bin, str(xacro_path or DEFAULT_XACRO),
         "only_left:=false", "fixed_legs:=false"],
        capture_output=True, check=True, text=True,
    ).stdout


def build_model(urdf_xml):
    """Build pinocchio Model + Data and a per-joint axis lookup (local frame).

    Pinocchio's joint-axis API depends on the joint template class. Parsing
    the URDF directly with urdf_parser_py for axes is dirt-cheap and
    unambiguous, so do that and use Pinocchio only for FK + gravity.
    """
    model = pin.buildModelFromXML(urdf_xml)
    data = pin.Data(model)

    # Local axes per joint name from the URDF (axis is in the parent-link
    # frame for revolute joints).
    from urdf_parser_py.urdf import URDF
    robot = URDF.from_xml_string(urdf_xml.encode("utf-8"))
    axes_local = {}
    for j in robot.joints:
        if j.type in ("revolute", "continuous", "prismatic"):
            a = np.asarray(j.axis or (1.0, 0.0, 0.0), dtype=float)
            n = np.linalg.norm(a)
            axes_local[j.name] = a / n if n > 0 else a

    return model, data, axes_local


def build_q_vector(model, pose):
    """Map a {joint_name: angle_rad} dict into pinocchio's q vector.

    Missing joints default to 0 (the URDF reference). Joints not in the
    Pinocchio model (e.g. typos) are silently skipped — they'd be flagged by
    the controller's own roster check on activation.
    """
    q = np.zeros(model.nq)
    for name, val in pose.items():
        if model.existJointName(name):
            jid = model.getJointId(name)
            q[model.joints[jid].idx_q] = float(val)
    return q


# ── Chain quantities at a pose ──────────────────────────────────────────────

def chain_snapshot(model, data, axes_local, q, roster=JOINT_ROSTER):
    """Compute per-joint kinematic + gravity quantities at the given q.

    Returns a list of dicts with keys: name, pivot_world, axis_world,
    a_horiz, mass, com_world, tau_g.
    """
    pin.forwardKinematics(model, data, q)
    pin.computeGeneralizedGravity(model, data, q)

    rows = []
    for name in roster:
        if not model.existJointName(name):
            rows.append({"name": name, "missing": True})
            continue
        jid = model.getJointId(name)
        joint = model.joints[jid]
        oMi = data.oMi[jid]

        # Axis in world: rotation of the joint's frame applied to the URDF
        # local axis. For revolute joints in pinocchio the joint frame's z'
        # axis is the rotation axis when JointModelRZ — but the URDF axis
        # gives us the canonical orientation directly.
        axis_local = axes_local.get(name, np.array([1.0, 0.0, 0.0]))
        axis_world = oMi.rotation @ axis_local
        a_horiz = float(np.sqrt(max(0.0, 1.0 - axis_world[2] ** 2)))

        # Pinocchio attaches the child link's body inertia to the joint.
        inertia = model.inertias[jid]
        mass = float(inertia.mass)
        com_world = oMi.act(inertia.lever)  # SE(3) action on the COM offset

        rows.append({
            "name": name,
            "pivot_world": np.array(oMi.translation),
            "axis_world": np.asarray(axis_world),
            "a_horiz": a_horiz,
            "mass": mass,
            "com_world": np.asarray(com_world),
            "tau_g": float(data.g[joint.idx_v]),
            "v_idx": int(joint.idx_v),
            "missing": False,
        })
    return rows


def sensitivity_matrix(model, data, axes_local, q, roster=JOINT_ROSTER,
                       epsilon=1e-4):
    """Numeric ∂tau_g / ∂q matrix over the roster.

    Returns a (N, N) ndarray indexed [j, k]: how much gravity torque on
    joint j changes per unit change in joint k, evaluated at q via central
    difference. Diagonal entries are how much each joint's own gravity load
    changes as the joint itself moves (also computable analytically, but the
    numeric path is uniform and uses Pinocchio's RNEA either way).
    """
    n = len(roster)
    q_idx = []
    v_idx = []
    for name in roster:
        if not model.existJointName(name):
            q_idx.append(-1); v_idx.append(-1); continue
        jid = model.getJointId(name)
        q_idx.append(model.joints[jid].idx_q)
        v_idx.append(model.joints[jid].idx_v)

    sens = np.zeros((n, n))
    for k in range(n):
        if q_idx[k] < 0:
            continue
        q_plus = q.copy(); q_plus[q_idx[k]] += epsilon
        q_minus = q.copy(); q_minus[q_idx[k]] -= epsilon
        pin.computeGeneralizedGravity(model, data, q_plus)
        g_plus = np.array([
            data.g[v_idx[j]] if v_idx[j] >= 0 else 0.0 for j in range(n)
        ])
        pin.computeGeneralizedGravity(model, data, q_minus)
        g_minus = np.array([
            data.g[v_idx[j]] if v_idx[j] >= 0 else 0.0 for j in range(n)
        ])
        sens[:, k] = (g_plus - g_minus) / (2.0 * epsilon)
    return sens


# ── Output formatting ───────────────────────────────────────────────────────

def _short(name):
    """Compact joint label for tables (~16 chars)."""
    return (name
        .replace("_joint", "")
        .replace("shoulder", "sh")
        .replace("elbow", "el")
        .replace("wrist", "wr")
        .replace("ankle", "an")
        .replace("hip", "hp")
        .replace("knee", "kn")
        .replace("waist", "wa"))


def format_kinematics_table(rows):
    """Per-joint world-frame axis, pivot, COM, mass, tau_g."""
    out = []
    out.append("  KINEMATICS + GRAVITY @ POSE")
    out.append("  " + "─" * 110)
    out.append(
        f"  {'joint':32s} "
        f"{'axis_world (x,y,z)':24s} "
        f"{'|a_h|':>6s}  "
        f"{'pivot (x,y,z)':20s} "
        f"{'m':>5s}  "
        f"{'tau_g':>8s}"
    )
    out.append(f"  {'':32s} {'':24s} {'':6s}  {'m':20s} {'kg':>5s}  {'N·m':>8s}")
    out.append("  " + "─" * 110)
    for r in rows:
        if r.get("missing"):
            out.append(f"  {r['name']:32s}  (not in URDF)")
            continue
        a = r["axis_world"]
        p = r["pivot_world"]
        out.append(
            f"  {r['name']:32s} "
            f"({a[0]:+5.2f},{a[1]:+5.2f},{a[2]:+5.2f}) "
            f"{r['a_horiz']:>6.3f}  "
            f"({p[0]:+5.2f},{p[1]:+5.2f},{p[2]:+5.2f}) "
            f"{r['mass']:>5.2f}  "
            f"{r['tau_g']:>+8.3f}"
        )
    return "\n".join(out)


def format_com_table(rows):
    """Per-link world-frame COM (the gravity lever's tip)."""
    out = []
    out.append("\n  LINK COM (world frame)")
    out.append("  " + "─" * 78)
    out.append(
        f"  {'joint':32s}  {'COM (x, y, z)':28s}  {'m':>6s}"
    )
    out.append(f"  {'':32s}  {'m':28s}  {'kg':>6s}")
    out.append("  " + "─" * 78)
    for r in rows:
        if r.get("missing"):
            continue
        c = r["com_world"]
        out.append(
            f"  {r['name']:32s}  "
            f"({c[0]:+6.3f}, {c[1]:+6.3f}, {c[2]:+6.3f})  "
            f"{r['mass']:>6.2f}"
        )
    return "\n".join(out)


def format_sensitivity_top(sens, roster, top_n=4, threshold=0.05):
    """For each joint j, list the upstream joints k whose movement most
    affects tau_g[j]. Filters out tiny couplings to keep the output scannable.
    """
    out = []
    out.append("\n  SENSITIVITY: top-{} ∂tau_g_j / ∂q_k per joint (units: N·m / rad)"
               .format(top_n))
    out.append("  Only entries with |∂tau/∂q| > {:.2f} N·m/rad shown.".format(threshold))
    out.append("  " + "─" * 100)
    for j, jname in enumerate(roster):
        col = sens[j, :]
        # Argsort descending by absolute value
        order = np.argsort(-np.abs(col))
        bits = []
        for k in order[:top_n]:
            if abs(col[k]) < threshold:
                continue
            bits.append(f"{_short(roster[k]):>16s}={col[k]:+6.2f}")
        if not bits:
            bits = ["(no significant coupling)"]
        out.append(f"  {jname:32s}  " + "  ".join(bits))
    return "\n".join(out)


# ── Static mode ─────────────────────────────────────────────────────────────

def load_pose(path):
    if path is None:
        return {}
    p = Path(path)
    if not p.exists():
        sys.stderr.write(f"pose file not found: {path}\n")
        sys.exit(2)
    data = yaml.safe_load(p.read_text()) or {}
    if not isinstance(data, dict):
        sys.stderr.write(f"{path}: expected a mapping {{joint: angle_rad}}\n")
        sys.exit(2)
    return {str(k): float(v) for k, v in data.items()}


def cmd_static(args):
    urdf = load_urdf_xml(args.xacro, args.urdf)
    model, data, axes_local = build_model(urdf)
    pose = load_pose(args.pose)
    q = build_q_vector(model, pose)

    print(f"\n  Pose: {args.pose if args.pose else 'URDF reference (all zeros)'}")
    print(f"  Model: nq={model.nq}, nv={model.nv}, joints (incl. universe)={model.njoints}")
    print(f"  Gravity vector: [0, 0, -9.81] m/s²\n")

    rows = chain_snapshot(model, data, axes_local, q)
    print(format_kinematics_table(rows))
    print(format_com_table(rows))
    if not args.no_sensitivity:
        sens = sensitivity_matrix(model, data, axes_local, q)
        print(format_sensitivity_top(sens, JOINT_ROSTER,
                                     top_n=args.top_n,
                                     threshold=args.sens_threshold))
    print()


# ── Live mode ───────────────────────────────────────────────────────────────

def cmd_live(args):
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except ImportError as e:
        sys.stderr.write(f"rclpy not importable: {e}\nSource /opt/ros/jazzy/setup.bash first.\n")
        sys.exit(1)

    urdf = load_urdf_xml(args.xacro, args.urdf)
    model, data, axes_local = build_model(urdf)

    class Monitor(Node):
        def __init__(self):
            super().__init__("chain_debug_monitor")
            self._latest = None
            self.create_subscription(
                JointState, args.topic, self._on_state, 10
            )
            period = 1.0 / max(args.rate, 0.5)
            self.create_timer(period, self._render)
            self.get_logger().info(
                f"chain_debug live on {args.topic}, refresh {args.rate} Hz")

        def _on_state(self, msg):
            self._latest = msg

        def _render(self):
            if self._latest is None:
                return
            pose = {
                n: p for n, p in zip(self._latest.name, self._latest.position)
            }
            q = build_q_vector(model, pose)
            rows = chain_snapshot(model, data, axes_local, q)
            # Clear screen + home cursor.
            sys.stdout.write("\x1b[H\x1b[2J")
            sys.stdout.write(
                f"  chain_debug — live @ {args.topic}, refresh {args.rate} Hz "
                "(Ctrl-C to quit)\n\n"
            )
            sys.stdout.write(format_kinematics_table(rows))
            sys.stdout.write("\n")
            sys.stdout.write(format_com_table(rows))
            sys.stdout.write("\n")
            sys.stdout.flush()

    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ── CLI ─────────────────────────────────────────────────────────────────────

def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--xacro", default=None,
                        help=f"Top-level xacro (default: {DEFAULT_XACRO})")
    common.add_argument("--urdf", default=None,
                        help="Pre-expanded URDF (bypasses xacro).")

    p_static = sub.add_parser("static", parents=[common],
                              help="one-shot snapshot at a pose YAML")
    p_static.add_argument("--pose", default=None,
                          help="YAML {joint: angle_rad}; default = all zeros")
    p_static.add_argument("--no-sensitivity", action="store_true",
                          help="skip the ∂tau_g/∂q sensitivity matrix")
    p_static.add_argument("--top-n", type=int, default=4,
                          help="per-joint top-N upstream couplings to show "
                               "(default 4)")
    p_static.add_argument("--sens-threshold", type=float, default=0.05,
                          help="hide sensitivity entries below this "
                               "|N·m/rad| (default 0.05)")
    p_static.set_defaults(func=cmd_static)

    p_live = sub.add_parser("live", parents=[common],
                            help="subscribe /joint_states_raw, refresh @ 5 Hz")
    p_live.add_argument("--topic", default="/joint_states_raw",
                        help="JointState topic (default: /joint_states_raw)")
    p_live.add_argument("--rate", type=float, default=5.0,
                        help="refresh rate Hz (default 5)")
    p_live.set_defaults(func=cmd_live)

    args = p.parse_args(argv)
    if args.xacro and args.urdf:
        p.error("--xacro and --urdf are mutually exclusive")
    args.func(args)


if __name__ == "__main__":
    main()
