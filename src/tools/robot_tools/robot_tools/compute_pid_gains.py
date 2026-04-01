#!/usr/bin/env python3
"""Compute physics-based PID gains from URDF inertial data and actuator specs.

Uses PyKDL's mass matrix computation to determine per-joint effective inertia,
then derives Kp/Kd gains from target bandwidth and damping ratio. Accounts for
reflected rotor inertia and ankle linkage force multiplication.

Gain formula:
    J_total  = J_load (from KDL H[i,i]) + J_motor * N_eff^2
    Kp_Nm    = J_total * omega_n^2
    Kd_Nm    = 2 * zeta * J_total * omega_n
    Kp_effort = Kp_Nm / Nm_per_effort
    Kd_effort = Kd_Nm / Nm_per_effort

where Nm_per_effort = 0.5 * Kt (module torque constant at output).

Usage:
    ros2 run robot_tools compute_pid_gains --verbose
    ros2 run robot_tools compute_pid_gains --output gains.yaml
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml

from robot_tools.kdl_helpers import build_chain, mass_matrix_diagonal

# ═══════════════════════════════════════════════════════════════════════════════
# Motor specifications — from docs/motor_specs.md (MyActuator RMD X V4)
# ═══════════════════════════════════════════════════════════════════════════════

MOTOR_SPECS = {
    "X4": {
        "gear_ratio": 36,
        "J_motor": 0.00003,       # kg*m^2 (0.3 Kg*cm^2 from datasheet)
        "Kt": 1.9,                # Nm/A at output (module torque constant)
        "rated_torque": 10.5,     # Nm
        "peak_torque": 34.0,      # Nm
        "friction": 1.14,         # Nm (back drive torque)
    },
    "X6": {
        "gear_ratio": 19.612,
        "J_motor": 0.000066,      # kg*m^2 (0.66 Kg*cm^2)
        "Kt": 2.1,
        "rated_torque": 20.0,
        "peak_torque": 60.0,
        "friction": 1.6,
    },
    "X8": {
        "gear_ratio": 19.612,
        "J_motor": 0.00015,       # kg*m^2 (1.5 Kg*cm^2)
        "Kt": 2.4,
        "rated_torque": 43.0,
        "peak_torque": 120.0,
        "friction": 3.21,
    },
}

# Effort unit conversion:
#   effort * 50 = raw PDO = per-mille of rated current (10A)
#   So effort 1.0 = 0.5A through the motor
#   Nm_per_effort = 0.5 * Kt

# ═══════════════════════════════════════════════════════════════════════════════
# Ankle linkage — eccentric-pushrod mechanism (2x X4 motors per ankle)
# From actuator_properties.xacro:
#   eccentric_radius = 25 mm, pitch_lever = 100 mm, roll_lever = 30 mm
#   force_mult = 2 * lever_arm / eccentric_radius
# ═══════════════════════════════════════════════════════════════════════════════

ANKLE_LINKAGE = {
    "pitch": {
        "force_mult": 8.0,       # 2 * 0.100 / 0.025
    },
    "roll": {
        "force_mult": 2.4,       # 2 * 0.030 / 0.025
    },
}

# ═══════════════════════════════════════════════════════════════════════════════
# Chain definitions — one per kinematic chain
# ═══════════════════════════════════════════════════════════════════════════════


@dataclass
class JointSpec:
    name: str
    motor: str
    ankle_func: Optional[str] = None  # "pitch" or "roll" for linkage joints


@dataclass
class ChainDef:
    name: str
    root_link: str
    tip_link: str
    joints: list[JointSpec] = field(default_factory=list)


CHAIN_DEFS = [
    ChainDef(
        name="left_arm",
        root_link="urdf_simplified_torso",
        tip_link="left_hand_base_link",
        joints=[
            JointSpec("left_shoulder_pitch_joint_X6", "X6"),
            JointSpec("left_shoulder_roll_joint_X6", "X6"),
            JointSpec("left_shoulder_yaw_joint_X4", "X4"),
            JointSpec("left_elbow_pitch_joint_X6", "X6"),
            JointSpec("left_wrist_yaw_joint_X4", "X4"),
            JointSpec("left_wrist_roll_joint_X4", "X4"),
        ],
    ),
    ChainDef(
        name="right_arm",
        root_link="urdf_simplified_torso",
        tip_link="right_hand_base_link",
        joints=[
            JointSpec("right_shoulder_pitch_joint_X6", "X6"),
            JointSpec("right_shoulder_roll_joint_X6", "X6"),
            JointSpec("right_shoulder_yaw_joint_X4", "X4"),
            JointSpec("right_elbow_pitch_joint_X6", "X6"),
            JointSpec("right_wrist_yaw_joint_X4", "X4"),
            JointSpec("right_wrist_roll_joint_X4", "X4"),
        ],
    ),
    ChainDef(
        name="left_leg",
        root_link="urdf_base",
        tip_link="urdf_foot_assembly",
        joints=[
            JointSpec("left_hip_pitch_joint_X8", "X8"),
            JointSpec("left_hip_roll_joint_X8", "X8"),
            JointSpec("left_hip_yaw_joint_X8", "X8"),
            JointSpec("left_knee_joint_X8", "X8"),
            JointSpec("left_ankle_pitch_joint_X4", "X4", ankle_func="pitch"),
            JointSpec("left_ankle_roll_joint_X4", "X4", ankle_func="roll"),
        ],
    ),
    ChainDef(
        name="right_leg",
        root_link="urdf_base",
        tip_link="urdf_foot_assembly_2",
        joints=[
            JointSpec("right_hip_pitch_joint_X8", "X8"),
            JointSpec("right_hip_roll_joint_X8", "X8"),
            JointSpec("right_hip_yaw_joint_X8", "X8"),
            JointSpec("right_knee_joint_X8", "X8"),
            # Right leg has ankle roll BEFORE pitch in URDF chain
            JointSpec("right_ankle_roll_joint_X4", "X4", ankle_func="roll"),
            JointSpec("right_ankle_pitch_joint_X4", "X4", ankle_func="pitch"),
        ],
    ),
]

# Waist is a special case — single joint connecting urdf_base → urdf_simplified_torso
# KDL can't compute its downstream inertia because arms/legs branch independently.
WAIST_JOINT = JointSpec("waist_yaw_joint_X8", "X8")
# Estimated Izz of upper body about yaw axis:
#   torso (5.9 kg) + 2 arms (~5.1 kg each) + hands (~0.53 kg each) ≈ 17.2 kg
#   Approximate as distributed mass at ~0.2 m radius: Izz ≈ m * r^2 / 2
WAIST_ESTIMATED_J_LOAD = 0.35  # kg*m^2 (conservative estimate)

# ═══════════════════════════════════════════════════════════════════════════════
# Gain computation
# ═══════════════════════════════════════════════════════════════════════════════


@dataclass
class JointGains:
    name: str
    motor: str
    J_load: float          # kg*m^2 — from KDL mass matrix diagonal
    J_reflected: float     # kg*m^2 — J_motor * N_eff^2
    J_total: float         # J_load + J_reflected
    Kp_Nm: float           # Nm/rad
    Kd_Nm: float           # Nm*s/rad
    Kp_effort: float       # effort units / rad
    Kd_effort: float       # effort units * s / rad
    Ki_effort: float       # effort units / (rad * s)
    Nm_per_effort: float
    omega_n: float
    zeta: float
    # Sanity checks
    torque_at_half_rad: float   # Nm when error = 0.5 rad
    peak_torque: float          # motor peak torque Nm
    saturates: bool
    is_estimated: bool = False  # True for waist (manual estimate)


def compute_joint_gains(
    j_load: float,
    spec: JointSpec,
    omega_n: float,
    zeta: float,
) -> JointGains:
    """Compute PID gains for a single joint."""
    motor = MOTOR_SPECS[spec.motor]
    N = motor["gear_ratio"]
    J_motor = motor["J_motor"]
    Kt = motor["Kt"]

    # Effort-to-torque conversion at the joint
    Nm_per_effort = 0.5 * Kt

    # For ankle linkage joints, the linkage multiplies force
    if spec.ankle_func is not None:
        linkage = ANKLE_LINKAGE[spec.ankle_func]
        N_eff = N * linkage["force_mult"]
        Nm_per_effort *= linkage["force_mult"]
        peak = motor["peak_torque"] * linkage["force_mult"]
    else:
        N_eff = N
        peak = motor["peak_torque"]

    J_reflected = J_motor * N_eff ** 2
    J_total = j_load + J_reflected

    Kp_Nm = J_total * omega_n ** 2
    Kd_Nm = 2.0 * zeta * J_total * omega_n

    Kp_effort = Kp_Nm / Nm_per_effort
    Kd_effort = Kd_Nm / Nm_per_effort

    torque_at_half = Kp_Nm * 0.5
    saturates = torque_at_half > peak

    return JointGains(
        name=spec.name,
        motor=spec.motor,
        J_load=j_load,
        J_reflected=J_reflected,
        J_total=J_total,
        Kp_Nm=Kp_Nm,
        Kd_Nm=Kd_Nm,
        Kp_effort=Kp_effort,
        Kd_effort=Kd_effort,
        Ki_effort=0.0,
        Nm_per_effort=Nm_per_effort,
        omega_n=omega_n,
        zeta=zeta,
        torque_at_half_rad=torque_at_half,
        peak_torque=peak,
        saturates=saturates,
    )


# ═══════════════════════════════════════════════════════════════════════════════
# URDF generation
# ═══════════════════════════════════════════════════════════════════════════════

def find_xacro_path() -> Optional[Path]:
    """Auto-detect the full_robot.urdf.xacro in the workspace."""
    candidates = [
        Path(__file__).resolve().parents[3]  # robot_tools → tools → src → ws
        / "src" / "robot_description" / "full_robot_description"
        / "urdf" / "full_robot.urdf.xacro",
    ]
    # Also try from cwd
    cwd = Path.cwd()
    candidates.append(
        cwd / "src" / "robot_description" / "full_robot_description"
        / "urdf" / "full_robot.urdf.xacro"
    )
    for c in candidates:
        if c.is_file():
            return c
    return None


def generate_urdf(xacro_path: Path) -> str:
    """Run xacro to produce URDF string with all joints as revolute."""
    result = subprocess.run(
        [
            "xacro", str(xacro_path),
            "only_left:=false",
            "fixed_legs:=false",
        ],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"xacro failed (exit {result.returncode}):\n{result.stderr}"
        )
    return result.stdout


# ═══════════════════════════════════════════════════════════════════════════════
# Evaluation configurations
# ═══════════════════════════════════════════════════════════════════════════════

def make_configs(n_joints: int, chain_name: str) -> list[tuple[str, list[float]]]:
    """Generate evaluation configurations for a chain.

    The mass matrix H[i,i] depends on downstream joint angles, not the
    joint's own angle. To capture worst-case inertia we must vary each
    downstream joint. Key insight: varying q[0] (base joint) has NO
    effect — it just rotates the whole chain.

    Returns list of (config_name, joint_positions) tuples.
    """
    zeros = [0.0] * n_joints

    if "arm" in chain_name:
        # Arm extended forward: shoulder roll out, elbow straight
        extended = list(zeros)
        if n_joints >= 2:
            extended[1] = math.pi / 2   # shoulder roll — arm out to side
        # Elbow bent: changes inertia seen by shoulder/yaw
        elbow_bent = list(zeros)
        if n_joints >= 4:
            elbow_bent[3] = math.pi / 3  # elbow at 60 deg
        # Mid-range: all downstream joints exercised
        mid = [0.0] + [0.4] * (n_joints - 1)
        return [
            ("zero", zeros),
            ("arm_out", extended),
            ("elbow_bent", elbow_bent),
            ("mid", mid),
        ]
    elif "leg" in chain_name:
        # Knee bent: changes inertia for hip joints
        bent = list(zeros)
        if n_joints >= 4:
            bent[3] = math.pi / 3  # knee at 60 deg
        # Hip yaw rotated: changes hip pitch/roll inertia
        yaw_rotated = list(zeros)
        if n_joints >= 3:
            yaw_rotated[2] = math.pi / 4  # hip yaw at 45 deg
        return [("zero", zeros), ("knee_bent", bent), ("hip_yaw_rot", yaw_rotated)]
    else:
        return [("zero", zeros)]


# ═══════════════════════════════════════════════════════════════════════════════
# Existing gains loading (for comparison)
# ═══════════════════════════════════════════════════════════════════════════════

def load_existing_gains(yaml_path: Path) -> dict[str, dict]:
    """Load gains from controllers_effort.yaml format."""
    with open(yaml_path) as f:
        data = yaml.safe_load(f)

    gains_section = (
        data.get("whole_body_controller", {})
        .get("ros__parameters", {})
        .get("gains", {})
    )
    return gains_section


def find_existing_gains_file() -> Optional[Path]:
    """Auto-detect controllers_effort.yaml."""
    candidates = [
        Path(__file__).resolve().parents[3]
        / "src" / "simulation" / "full_robot_gazebo"
        / "config" / "controllers_effort.yaml",
    ]
    cwd = Path.cwd()
    candidates.append(
        cwd / "src" / "simulation" / "full_robot_gazebo"
        / "config" / "controllers_effort.yaml"
    )
    for c in candidates:
        if c.is_file():
            return c
    return None


# ═══════════════════════════════════════════════════════════════════════════════
# Output formatting
# ═══════════════════════════════════════════════════════════════════════════════

def print_results(
    all_gains: list[JointGains],
    existing: Optional[dict[str, dict]],
    verbose: bool,
):
    """Print results table with optional comparison to existing gains."""
    # Header
    if existing:
        print(f"\n{'Joint':<38s} {'Motor':>5s} {'J_load':>7s} {'J_refl':>7s} "
              f"{'J_tot':>7s} {'Kp':>7s} {'Kp_old':>7s} {'Kd':>7s} {'Kd_old':>7s} "
              f"{'Sat':>4s}")
        print("-" * 110)
    else:
        print(f"\n{'Joint':<38s} {'Motor':>5s} {'J_load':>7s} {'J_refl':>7s} "
              f"{'J_tot':>7s} {'Kp':>7s} {'Kd':>7s} {'Sat':>4s}")
        print("-" * 90)

    for g in all_gains:
        sat_flag = "!!" if g.saturates else "ok"
        est_flag = " ~" if g.is_estimated else ""

        if existing:
            old = existing.get(g.name, {})
            old_kp = old.get("p", 0.0)
            old_kd = old.get("d", 0.0)
            print(
                f"{g.name:<38s} {g.motor:>5s} "
                f"{g.J_load:>7.4f}{est_flag} {g.J_reflected:>7.4f} {g.J_total:>7.4f} "
                f"{g.Kp_effort:>7.1f} {old_kp:>7.1f} "
                f"{g.Kd_effort:>7.2f} {old_kd:>7.2f} "
                f"{sat_flag:>4s}"
            )
        else:
            print(
                f"{g.name:<38s} {g.motor:>5s} "
                f"{g.J_load:>7.4f}{est_flag} {g.J_reflected:>7.4f} {g.J_total:>7.4f} "
                f"{g.Kp_effort:>7.1f} {g.Kd_effort:>7.2f} {sat_flag:>4s}"
            )

    # Verbose: show additional diagnostics
    if verbose:
        print(f"\n{'--- Diagnostics ---':^90}")
        print(f"{'Joint':<38s} {'omega_n':>7s} {'zeta':>5s} "
              f"{'Kp_Nm':>8s} {'Kd_Nm':>8s} {'tau@0.5r':>9s} {'peak':>7s} "
              f"{'Nm/eff':>7s}")
        print("-" * 100)
        for g in all_gains:
            print(
                f"{g.name:<38s} {g.omega_n:>7.1f} {g.zeta:>5.2f} "
                f"{g.Kp_Nm:>8.2f} {g.Kd_Nm:>8.2f} "
                f"{g.torque_at_half_rad:>9.2f} {g.peak_torque:>7.1f} "
                f"{g.Nm_per_effort:>7.3f}"
            )

    # Warnings
    print()
    for g in all_gains:
        if g.saturates:
            print(f"  WARNING: {g.name} — torque at 0.5 rad error "
                  f"({g.torque_at_half_rad:.1f} Nm) exceeds peak "
                  f"({g.peak_torque:.1f} Nm). Reduce omega_n.")
        if g.is_estimated:
            print(f"  NOTE: {g.name} — J_load is estimated (not from KDL). "
                  f"Verify on hardware.")


def write_yaml_output(all_gains: list[JointGains], output_path: Path):
    """Write gains in controllers_effort.yaml format."""
    gains_dict = {}
    for g in all_gains:
        entry = {
            "p": round(g.Kp_effort, 1),
            "d": round(g.Kd_effort, 1),
            "i": round(g.Ki_effort, 1),
        }
        gains_dict[g.name] = entry

    # Build full controller config structure
    joint_names = [g.name for g in all_gains]
    config = {
        "controller_manager": {
            "ros__parameters": {
                "update_rate": 200,
                "joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster",
                },
                "whole_body_controller": {
                    "type": "joint_trajectory_controller/JointTrajectoryController",
                },
            },
        },
        "whole_body_controller": {
            "ros__parameters": {
                "joints": joint_names,
                "command_interfaces": ["effort"],
                "state_interfaces": ["position", "velocity"],
                "allow_nonzero_velocity_at_trajectory_end": True,
                "allow_integration_in_goal_trajectories": True,
                "open_loop_control": False,
                "constraints": {
                    "stopped_velocity_tolerance": 0.0,
                    "goal_time": 0.0,
                },
                "gains": gains_dict,
            },
        },
    }

    with open(output_path, "w") as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    print(f"\nGains written to: {output_path}")


# ═══════════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Compute physics-based PID gains from URDF and actuator specs.",
    )
    parser.add_argument(
        "--xacro", type=Path, default=None,
        help="Path to full_robot.urdf.xacro (auto-detected if omitted)",
    )
    parser.add_argument(
        "--omega-n-arm", type=float, default=30.0,
        help="Natural frequency for arm joints (rad/s, default: 30)",
    )
    parser.add_argument(
        "--omega-n-leg", type=float, default=40.0,
        help="Natural frequency for leg joints (rad/s, default: 40)",
    )
    parser.add_argument(
        "--omega-n-waist", type=float, default=35.0,
        help="Natural frequency for waist (rad/s, default: 35)",
    )
    parser.add_argument(
        "--zeta", type=float, default=1.0,
        help="Damping ratio (default: 1.0 = critically damped)",
    )
    parser.add_argument(
        "--output", type=Path, default=None,
        help="Write gains YAML to file",
    )
    parser.add_argument(
        "--compare", type=Path, default=None,
        help="Compare with existing gains YAML file",
    )
    parser.add_argument(
        "--no-compare", action="store_true",
        help="Disable comparison with existing gains (default: comparison enabled)",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="Show mass matrix details and intermediate values",
    )
    args = parser.parse_args()

    # --- Find and generate URDF ---
    xacro_path = args.xacro or find_xacro_path()
    if xacro_path is None or not xacro_path.is_file():
        print("ERROR: Cannot find full_robot.urdf.xacro. Use --xacro to specify.",
              file=sys.stderr)
        return 1

    print(f"Generating URDF from: {xacro_path}")
    try:
        urdf_string = generate_urdf(xacro_path)
    except RuntimeError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1

    print(f"URDF generated ({len(urdf_string)} bytes)")

    # --- Compute gains for each chain ---
    all_gains: list[JointGains] = []

    for chain_def in CHAIN_DEFS:
        if "arm" in chain_def.name:
            omega_n = args.omega_n_arm
        else:
            omega_n = args.omega_n_leg

        print(f"\nProcessing chain: {chain_def.name} "
              f"({chain_def.root_link} -> {chain_def.tip_link}, omega_n={omega_n})")

        try:
            chain = build_chain(urdf_string, chain_def.root_link, chain_def.tip_link)
        except ValueError as e:
            print(f"  ERROR building chain: {e}", file=sys.stderr)
            continue

        n_joints = chain.getNrOfJoints()
        n_expected = len(chain_def.joints)
        print(f"  KDL chain: {n_joints} DOFs, {chain.getNrOfSegments()} segments")

        if n_joints != n_expected:
            print(f"  WARNING: Expected {n_expected} DOFs but got {n_joints}. "
                  f"Chain may include unexpected joints.", file=sys.stderr)

        # Evaluate at multiple configurations, take max inertia per joint
        configs = make_configs(n_joints, chain_def.name)
        max_diag = [0.0] * n_joints

        for config_name, positions in configs:
            try:
                diag = mass_matrix_diagonal(chain, positions)
            except (RuntimeError, ValueError) as e:
                print(f"  WARNING: Mass matrix at '{config_name}' failed: {e}")
                continue

            if args.verbose:
                diag_str = ", ".join(f"{d:.6f}" for d in diag)
                print(f"  H_diag @ {config_name}: [{diag_str}]")

            for i in range(n_joints):
                max_diag[i] = max(max_diag[i], diag[i])

        # Compute gains per joint
        for i, jspec in enumerate(chain_def.joints):
            if i < len(max_diag):
                gains = compute_joint_gains(max_diag[i], jspec, omega_n, args.zeta)
                all_gains.append(gains)

    # --- Waist (special case) ---
    print(f"\nProcessing waist (estimated J_load={WAIST_ESTIMATED_J_LOAD:.3f} kg*m^2)")
    waist_gains = compute_joint_gains(
        WAIST_ESTIMATED_J_LOAD, WAIST_JOINT, args.omega_n_waist, args.zeta,
    )
    waist_gains.is_estimated = True

    # Insert waist after right arm (index 12), matching controllers_effort.yaml order
    # Order: left_arm(0-5), right_arm(6-11), waist(12), left_leg(13-18), right_leg(19-24)
    arm_count = sum(1 for g in all_gains if "arm" in g.name or "shoulder" in g.name
                    or "elbow" in g.name or "wrist" in g.name)
    all_gains.insert(arm_count, waist_gains)

    # --- Load existing gains for comparison ---
    existing = None
    if not args.no_compare:
        compare_path = args.compare or find_existing_gains_file()
        if compare_path and compare_path.is_file():
            existing = load_existing_gains(compare_path)
            print(f"\nComparing with: {compare_path}")

    # --- Print results ---
    print_results(all_gains, existing, args.verbose)

    # --- Write output ---
    if args.output:
        write_yaml_output(all_gains, args.output)

    # --- Summary ---
    n_saturated = sum(1 for g in all_gains if g.saturates)
    print(f"\nTotal: {len(all_gains)} joints computed, "
          f"{n_saturated} saturation warnings")

    return 0


if __name__ == "__main__":
    sys.exit(main())
