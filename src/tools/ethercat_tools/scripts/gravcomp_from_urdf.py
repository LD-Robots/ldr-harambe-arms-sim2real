#!/usr/bin/env python3
"""
Derive gravity-compensation data from the URDF for the PVT controller.

The whole-body PVT controller's gravity feedforward term is

    tau_g = mgl[j] * sin(q[j])

— a single-pendulum approximation about each joint's local angle. To feed
this term the controller needs `mgl[j]` populated; the YAML in
`src/bringup/robot_bringup/config/controllers_pvt.yaml` ships them as a
single `[0.0]` placeholder, with the bringup comment explicit that "these
need URDF-derived values before enabling ff_*."

Two subcommands:

  emit      Walk the URDF tree and write a YAML containing every joint's
            parent/child/origin/axis plus every distal link's mass, COM
            offset, and inertia tensor. This is structured data for a
            future sidecar or controller extension that will do full FK
            gravity comp at runtime — see the plan file for context.

  evaluate  At a caller-supplied pose (default: URDF reference, all zeros),
            compute the peak gravity torque each joint sees as its own
            angle sweeps a full revolution with every other joint held at
            the reference. Print a table and a paste-ready snippet for
            controllers_pvt.yaml. Peak |tau_g| is the right `mgl[j]`
            constant for the single-pendulum model anchored at that pose
            — locally correct, degrades as the chain articulates away
            from the reference.

Why FK by hand instead of PyKDL: kdl_parser_py is not available on Jazzy
(noted in `arm_real_bringup/scripts/gravity_comp_node.py:158`), the tree
has branching (waist_yaw fans out to both arms), and the math is more
transparent as four lines of numpy than as a KDL Chain build per joint.

The leg joints get `ff_gravity = false` in the emitted snippet — see the
"legs" comment block below.

Usage:
    python gravcomp_from_urdf.py emit --out /tmp/gravcomp.yaml
    python gravcomp_from_urdf.py evaluate
    python gravcomp_from_urdf.py evaluate --pose stance.yaml
"""

import argparse
import shutil
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import yaml

try:
    from urdf_parser_py.urdf import URDF
except ImportError as e:
    sys.stderr.write(
        f"urdf_parser_py not available: {e}\n"
        "Install with: sudo apt install python3-urdf-parser-py\n"
    )
    raise


# ── Joint roster (matches controllers_pvt.yaml verbatim) ─────────────────────
# Note: right-leg ankle order is roll-then-pitch, opposite the left leg's
# pitch-then-roll. That mirror asymmetry is in the URDF — preserve it.
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

# Legs: gravity feedforward as a sum of downstream link weights is only
# correct for an open chain. In stance the foot is on the floor — ground
# reaction forces flow up through the leg, so the joint torques needed to
# hold a pose come from full inverse dynamics with contact, NOT from the
# dangling-mass formula. Naive feedforward in stance would push the leg
# outward. We compute the numbers (they're useful as references) but
# emit `ff_gravity = false` for every leg joint in the snippet.
LEG_JOINTS = frozenset({
    "left_hip_pitch_joint_X8", "left_hip_roll_joint_X8",
    "left_hip_yaw_joint_X8", "left_knee_joint_X8",
    "left_ankle_pitch_joint_X4", "left_ankle_roll_joint_X4",
    "right_hip_pitch_joint_X8", "right_hip_roll_joint_X8",
    "right_hip_yaw_joint_X8", "right_knee_joint_X8",
    "right_ankle_roll_joint_X4", "right_ankle_pitch_joint_X4",
})

GRAVITY = np.array([0.0, 0.0, -9.81])   # world frame, m/s^2

# Default URDF xacro — resolved relative to the script. Works for source-tree
# invocation; install-tree callers should pass --xacro explicitly.
REPO_SRC = Path(__file__).resolve().parents[3]
DEFAULT_XACRO = (
    REPO_SRC
    / "robot_description/full_robot_description/urdf/full_robot.urdf.xacro"
)


# ── URDF math helpers ───────────────────────────────────────────────────────

def rpy_to_rotation(rpy):
    """ROS / URDF RPY (XYZ fixed-axis = ZYX intrinsic) -> 3x3 rotation."""
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return R_z @ R_y @ R_x


def make_transform(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_rotation(rpy)
    T[:3, 3] = xyz
    return T


def axis_angle_rotation(axis, angle):
    """Rodrigues' formula. `axis` is normalised internally."""
    a = np.asarray(axis, dtype=float)
    n = np.linalg.norm(a)
    if n == 0.0:
        return np.eye(3)
    a = a / n
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


def origin_xyz(origin):
    if origin is None or origin.xyz is None:
        return (0.0, 0.0, 0.0)
    return tuple(origin.xyz)


def origin_rpy(origin):
    if origin is None or origin.rpy is None:
        return (0.0, 0.0, 0.0)
    return tuple(origin.rpy)


def joint_axis(joint):
    if joint.axis is None:
        return (1.0, 0.0, 0.0)
    return tuple(joint.axis)


def joint_transform(joint, q):
    """Parent-link frame -> child-link frame for the given joint state."""
    T_origin = make_transform(origin_xyz(joint.origin), origin_rpy(joint.origin))
    if joint.type in ("revolute", "continuous"):
        rot = np.eye(4)
        rot[:3, :3] = axis_angle_rotation(joint_axis(joint), q)
        return T_origin @ rot
    if joint.type == "prismatic":
        axis = np.asarray(joint_axis(joint), dtype=float)
        axis = axis / np.linalg.norm(axis)
        trans = np.eye(4)
        trans[:3, 3] = axis * q
        return T_origin @ trans
    return T_origin   # fixed / floating treated as fixed


# ── Tree wrangling ──────────────────────────────────────────────────────────

def build_tree(robot):
    """Return (parent_joint_of_link, children_joints_of_link)."""
    parent = {}
    children = defaultdict(list)
    for j in robot.joints:
        parent[j.child] = j
        children[j.parent].append(j)
    return parent, children


def root_link(robot, parent_joint):
    """The link with no parent joint — should be `world` or `urdf_base`."""
    roots = [l.name for l in robot.links if l.name not in parent_joint]
    if len(roots) != 1:
        raise RuntimeError(f"expected exactly one root link, got {roots}")
    return roots[0]


def world_transform_of_link(link_name, parent_joint, q_by_joint):
    """4x4 world pose of `link_name` given a pose dict (missing joints -> 0)."""
    chain = []
    cur = link_name
    while cur in parent_joint:
        j = parent_joint[cur]
        chain.append(j)
        cur = j.parent
    T = np.eye(4)
    for j in reversed(chain):  # root → tip
        q = q_by_joint.get(j.name, 0.0)
        T = T @ joint_transform(j, q)
    return T


def subtree_links(start_link, children):
    """Every link reachable from `start_link` via outgoing joints (inclusive)."""
    seen = set()
    frontier = [start_link]
    while frontier:
        link = frontier.pop()
        if link in seen:
            continue
        seen.add(link)
        for j in children.get(link, []):
            frontier.append(j.child)
    return seen


# ── Gravity-comp math ───────────────────────────────────────────────────────

def joint_world_pivot_and_axis(joint, parent_joint, q_by_joint):
    """World position of joint axis and unit-axis direction at the pose."""
    T_parent = world_transform_of_link(joint.parent, parent_joint, q_by_joint)
    T_origin = make_transform(origin_xyz(joint.origin), origin_rpy(joint.origin))
    T_joint = T_parent @ T_origin
    pivot = T_joint[:3, 3]
    a_local = np.asarray(joint_axis(joint), dtype=float)
    a_local = a_local / np.linalg.norm(a_local)
    axis_world = T_joint[:3, :3] @ a_local
    return pivot, axis_world


def composite_downstream_com(joint, robot, parent_joint, children, q_by_joint):
    """Lumped (total_mass, COM_world) over every link strictly downstream of
    the joint at the supplied pose. Returns (0, zeros) if the subtree is
    massless."""
    link_map = {l.name: l for l in robot.links}
    subtree = subtree_links(joint.child, children)
    total = 0.0
    weighted = np.zeros(3)
    for name in subtree:
        link = link_map.get(name)
        if link is None or link.inertial is None:
            continue
        m = link.inertial.mass
        if m is None or m == 0.0:
            continue
        com_local = np.array([*origin_xyz(link.inertial.origin), 1.0])
        T = world_transform_of_link(name, parent_joint, q_by_joint)
        com_world = (T @ com_local)[:3]
        total += m
        weighted += m * com_world
    com = weighted / total if total > 0.0 else np.zeros(3)
    return total, com


def gravity_torque_about_joint(joint, robot, parent_joint, children, q_by_joint):
    """tau_g[j] at the given pose. Sums (r × m·g) · axis over the full subtree."""
    pivot, axis = joint_world_pivot_and_axis(joint, parent_joint, q_by_joint)
    link_map = {l.name: l for l in robot.links}
    subtree = subtree_links(joint.child, children)
    tau = 0.0
    for name in subtree:
        link = link_map.get(name)
        if link is None or link.inertial is None:
            continue
        m = link.inertial.mass
        if m is None or m == 0.0:
            continue
        com_local = np.array([*origin_xyz(link.inertial.origin), 1.0])
        T = world_transform_of_link(name, parent_joint, q_by_joint)
        com_world = (T @ com_local)[:3]
        r = com_world - pivot
        torque_vec = np.cross(r, m * GRAVITY)
        tau += float(np.dot(axis, torque_vec))
    return tau


def peak_mgl(joint, robot, parent_joint, children, q_by_joint):
    """Closed-form peak |tau_g(q_j)| as joint j sweeps a full revolution with
    every other joint held at q_by_joint.

    Derivation: under rotation of joint j by Δq about axis a, the whole
    subtree rigidly rotates by R_a(Δq) about the joint pivot. Let
    r = r_com_world - pivot. Decompose r = r_∥ + r_⊥ (parallel / perpendicular
    to a). Only r_⊥ rotates; its magnitude is invariant. The torque about a is

        tau = a · (r × F),  F = M · g_world

    Working through the algebra (a · (r_∥ × F) = 0 by triple product), the
    varying part has amplitude |r_⊥| · |F_⊥| where F_⊥ = F − (F·a)·a is the
    gravity component perpendicular to the axis. So

        peak |tau| = M · |g| · |r_⊥| · sqrt(1 − a_z²)

    If a is vertical (a_z = ±1) the joint can't couple gravity at all
    (waist_yaw is the obvious case) — formula returns 0. If a is horizontal
    the full M·g·|r_⊥| lever shows up.
    """
    pivot, axis = joint_world_pivot_and_axis(joint, parent_joint, q_by_joint)
    M, com = composite_downstream_com(
        joint, robot, parent_joint, children, q_by_joint)
    if M == 0.0:
        return 0.0, 0.0, 0.0, axis, com - pivot
    r = com - pivot
    r_perp = r - np.dot(r, axis) * axis
    r_perp_norm = float(np.linalg.norm(r_perp))
    horizontal_axis_fraction = float(np.sqrt(max(0.0, 1.0 - axis[2] ** 2)))
    peak = M * float(np.linalg.norm(GRAVITY)) * r_perp_norm * horizontal_axis_fraction
    return peak, M, r_perp_norm, axis, r


# ── URDF loading ────────────────────────────────────────────────────────────

def load_urdf(xacro_path=None, urdf_path=None, mappings=None):
    """Run xacro on `xacro_path` (or read `urdf_path`) and return a URDF object.

    `mappings` is a list of `name:=value` strings forwarded to xacro. We pass
    `only_left:=false fixed_legs:=false` by default so that waist_yaw and all
    leg joints come back as revolute, not fixed.
    """
    if urdf_path is not None:
        urdf_bytes = Path(urdf_path).read_bytes()
    else:
        if xacro_path is None:
            xacro_path = DEFAULT_XACRO
        xacro_bin = shutil.which("xacro")
        if xacro_bin is None:
            sys.stderr.write(
                "xacro not found on PATH. source /opt/ros/jazzy/setup.bash first.\n"
            )
            sys.exit(1)
        args = [xacro_bin, str(xacro_path)]
        if mappings:
            args.extend(mappings)
        result = subprocess.run(args, capture_output=True)
        if result.returncode != 0:
            sys.stderr.write(
                f"xacro failed (exit {result.returncode}):\n"
                f"  cmd: {' '.join(args)}\n"
                f"{result.stderr.decode('utf-8', errors='replace')}\n"
                "Hint: source the workspace first — both\n"
                "  /opt/ros/jazzy/setup.bash\n"
                "  <repo>/install/setup.bash\n"
                "must be on AMENT_PREFIX_PATH for xacro's $(find …) to resolve.\n"
            )
            sys.exit(result.returncode)
        urdf_bytes = result.stdout
    return URDF.from_xml_string(urdf_bytes)


def find_joint(robot, name):
    for j in robot.joints:
        if j.name == name:
            return j
    raise KeyError(name)


# ── Subcommand: emit ────────────────────────────────────────────────────────

def cmd_emit(args):
    """Write structured per-joint and per-link kinematic+inertial data."""
    robot = load_urdf(
        xacro_path=args.xacro,
        urdf_path=args.urdf,
        mappings=["only_left:=false", "fixed_legs:=false"],
    )
    parent_joint, _ = build_tree(robot)
    link_map = {l.name: l for l in robot.links}

    # Joints, in roster order
    joints_out = []
    referenced_links = set()
    for name in JOINT_ROSTER:
        j = find_joint(robot, name)
        joints_out.append({
            "name": j.name,
            "parent_link": j.parent,
            "child_link": j.child,
            "type": j.type,
            "origin_xyz": list(origin_xyz(j.origin)),
            "origin_rpy": list(origin_rpy(j.origin)),
            "axis": list(joint_axis(j)),
        })
        referenced_links.add(j.parent)
        referenced_links.add(j.child)

    # Walk every roster joint's downstream subtree to pick up all distal links
    # (including hand sub-chains hanging off the wrist).
    _, children = build_tree(robot)
    for name in JOINT_ROSTER:
        j = find_joint(robot, name)
        referenced_links.update(subtree_links(j.child, children))

    links_out = []
    for name in sorted(referenced_links):
        link = link_map.get(name)
        if link is None:
            continue
        entry = {"name": name}
        if link.inertial is not None:
            entry["mass"] = link.inertial.mass or 0.0
            entry["com_xyz"] = list(origin_xyz(link.inertial.origin))
            entry["com_rpy"] = list(origin_rpy(link.inertial.origin))
            inertia = link.inertial.inertia
            if inertia is not None:
                entry["inertia"] = {
                    "ixx": inertia.ixx or 0.0,
                    "ixy": inertia.ixy or 0.0,
                    "ixz": inertia.ixz or 0.0,
                    "iyy": inertia.iyy or 0.0,
                    "iyz": inertia.iyz or 0.0,
                    "izz": inertia.izz or 0.0,
                }
        else:
            entry["mass"] = 0.0
            entry["com_xyz"] = [0.0, 0.0, 0.0]
            entry["com_rpy"] = [0.0, 0.0, 0.0]
        links_out.append(entry)

    out = {
        "_generated_by": "src/tools/ethercat_tools/scripts/gravcomp_from_urdf.py",
        "_source_urdf": str(args.xacro or args.urdf or DEFAULT_XACRO),
        "joints": joints_out,
        "links": links_out,
    }

    dumped = yaml.safe_dump(out, sort_keys=False, default_flow_style=None)
    if args.out:
        Path(args.out).write_text(dumped)
        sys.stderr.write(f"wrote {args.out} ({len(joints_out)} joints, "
                         f"{len(links_out)} links)\n")
    else:
        sys.stdout.write(dumped)


# ── Subcommand: evaluate ────────────────────────────────────────────────────

def load_pose(path):
    """Pose YAML is just {joint_name: angle_rad}. Missing joints default to 0."""
    if path is None:
        return {}
    p = Path(path)
    if not p.exists():
        sys.stderr.write(
            f"pose file not found: {path}\n\n"
            "Create one with `evaluate --print-pose-template > my_pose.yaml`,\n"
            "edit the angles, then pass it via `--pose my_pose.yaml`.\n"
            "Or drop `--pose` entirely to evaluate at the URDF reference (all zeros).\n"
        )
        sys.exit(2)
    data = yaml.safe_load(p.read_text()) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a mapping {{joint: angle_rad}}")
    return {str(k): float(v) for k, v in data.items()}


def print_pose_template():
    """Dump a YAML pose template (every roster joint at 0.0 rad)."""
    sys.stdout.write(
        "# Pose template — pass to `evaluate --pose <file>`.\n"
        "# Units are radians. Missing joints default to 0.\n"
        "# The values below are the URDF reference pose (all zeros);\n"
        "# edit any subset to the stance you care about.\n"
    )
    for name in JOINT_ROSTER:
        sys.stdout.write(f"{name}: 0.0\n")


def cmd_evaluate(args):
    """Print per-joint gravity-comp summary at the reference pose."""
    if args.print_pose_template:
        print_pose_template()
        return
    robot = load_urdf(
        xacro_path=args.xacro,
        urdf_path=args.urdf,
        mappings=["only_left:=false", "fixed_legs:=false"],
    )
    parent_joint, children = build_tree(robot)
    pose = load_pose(args.pose)

    # Per-joint analysis
    rows = []
    for name in JOINT_ROSTER:
        j = find_joint(robot, name)
        peak, M, r_perp, axis, r = peak_mgl(
            j, robot, parent_joint, children, pose)
        tau = gravity_torque_about_joint(
            j, robot, parent_joint, children, pose)
        rows.append({
            "joint": name,
            "M_sub": M,
            "r_perp": r_perp,
            "a_z": float(axis[2]),
            "mgl_peak": peak,
            "tau_at_pose": tau,
            "is_leg": name in LEG_JOINTS,
        })

    # Pretty table
    sys.stdout.write(
        f"\nReference pose: {'all zeros' if not pose else args.pose}\n"
        f"Gravity:        {GRAVITY.tolist()} m/s^2\n\n"
    )
    header = (
        f"  {'joint':38s} {'M_sub':>8s} {'|r⊥|':>8s} {'|a_h|':>7s} "
        f"{'mgl_peak':>10s} {'tau@pose':>10s}  leg\n"
        f"  {'':38s} {'kg':>8s} {'m':>8s} {'':>7s} "
        f"{'N·m':>10s} {'N·m':>10s}\n"
    )
    sys.stdout.write(header)
    sys.stdout.write("  " + "─" * 96 + "\n")
    for r in rows:
        a_h = float(np.sqrt(max(0.0, 1.0 - r["a_z"] ** 2)))
        leg_mark = " ✓" if r["is_leg"] else ""
        sys.stdout.write(
            f"  {r['joint']:38s} {r['M_sub']:8.3f} {r['r_perp']:8.4f} "
            f"{a_h:7.3f} {r['mgl_peak']:10.4f} {r['tau_at_pose']:+10.4f}{leg_mark}\n"
        )
    sys.stdout.write("\n")

    # Paste-ready snippet for controllers_pvt.yaml
    mgl_vals = []
    ff_vals = []
    for r in rows:
        if r["is_leg"]:
            mgl_vals.append(0.0)
            ff_vals.append(False)
        else:
            # No rounding — preserve full IEEE-double precision so the YAML
            # round-trips back to the same number the script computed.
            mgl_vals.append(r["mgl_peak"])
            # Waist axis is vertical -> mgl_peak == 0 -> feedforward is a no-op.
            ff_vals.append(r["mgl_peak"] > 1e-9)

    sys.stdout.write(
        "# Paste into src/bringup/robot_bringup/config/controllers_pvt.yaml.\n"
        "# Leg joints stay at mgl=0, ff_gravity=false — stance-leg gravcomp\n"
        "# requires contact-aware control, not the dangling-mass formula.\n"
        "# comp_sign is left at 1.0; flip per-joint at bench time after\n"
        "# observing whether the joint drifts toward or away from gravity\n"
        "# when ff_gravity is enabled at zero PD gain.\n"
        "robot_pvt_controller:\n"
        "  ros__parameters:\n"
    )
    sys.stdout.write("    mgl: " + _fmt_yaml_list(mgl_vals) + "\n")
    sys.stdout.write("    ff_gravity: " + _fmt_yaml_list(ff_vals) + "\n")
    sys.stdout.write("    comp_sign: " + _fmt_yaml_list([1.0] * len(rows)) + "\n")


def _fmt_yaml_list(values):
    """Render a list on one line.

    Floats are emitted via repr() to keep full IEEE-double precision (the
    ROS 2 parameter loader requires double-typed YAML; `0` would type-infer
    as int and the controller's as_double_array() call would reject it).
    Booleans render as YAML true/false.
    """
    parts = []
    for v in values:
        if isinstance(v, bool):
            parts.append("true" if v else "false")
        elif isinstance(v, float):
            s = repr(v)
            # repr(1.0) -> "1.0" (good); repr(0.0) -> "0.0" (good).
            # No further massaging needed.
            parts.append(s)
        else:
            parts.append(repr(float(v)))
    return "[" + ", ".join(parts) + "]"


# ── CLI ─────────────────────────────────────────────────────────────────────

def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        "--xacro", default=None,
        help=f"Top-level xacro to expand (default: {DEFAULT_XACRO})",
    )
    common.add_argument(
        "--urdf", default=None,
        help="Pre-expanded URDF (bypasses xacro). Mutually exclusive with --xacro.",
    )

    p_emit = sub.add_parser("emit", parents=[common],
                            help="dump per-joint / per-link YAML")
    p_emit.add_argument("--out", default=None,
                        help="output file (default: stdout)")
    p_emit.set_defaults(func=cmd_emit)

    p_eval = sub.add_parser("evaluate", parents=[common],
                            help="compute mgl[j] at a reference pose")
    p_eval.add_argument(
        "--pose", default=None,
        help="YAML {joint_name: angle_rad}; missing joints default to 0",
    )
    p_eval.add_argument(
        "--print-pose-template", action="store_true",
        help="print a YAML template with every roster joint at 0.0 rad and exit",
    )
    p_eval.set_defaults(func=cmd_evaluate)

    args = p.parse_args(argv)
    if args.xacro and args.urdf:
        p.error("--xacro and --urdf are mutually exclusive")
    args.func(args)


if __name__ == "__main__":
    main()
