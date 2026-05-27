#!/usr/bin/env python3
"""
Recompute per-joint EtherCAT torque factors and patch the slave YAMLs.

CST/PVT torque on a MyActuator V4 drive is commanded as per-mille of rated
current (NOT rated torque — deviation from the CiA 402 standard, see
docs/tuning/motor_id_protocol.html §10.2). The forward chain is

    T_shaft = T_cmd_‰ · (factor_cmd / 1000) · I_rated · K_t

so the YAML's `command_interface: effort` factor on RxPDO 0x6071 must be

    factor_cmd   = 1000 / (K_t · I_rated)
    factor_state = 1   / factor_cmd        (state on TxPDO 0x6077)

This script is the single source of truth for that arithmetic. It is
intentionally run by hand — not wired into colcon — same as
`demo_joint_offset`. Re-run it whenever K_t or I_rated changes.

Usage:
    python recompute_torque_factors.py print
    python recompute_torque_factors.py patch --dry-run
    python recompute_torque_factors.py patch --apply
    python recompute_torque_factors.py patch --apply --force   # bypass guard
"""

import argparse
import re
import sys
from decimal import Decimal, getcontext
from pathlib import Path

# Decimal precision wide enough to keep all derived factors exact at IEEE
# 754 double granularity; the inputs (K_t, I_rated) carry ≤ 4 sig figs.
getcontext().prec = 28

# ─── Motor constants ──────────────────────────────────────────────────────────
#
# K_t — MyActuator RMD V4 datasheet (N·m/A, output-side per motor amp).
# NOT the "KT_OUT" column in All_Updated_Motors_by_Type.ods. That column
# (X4=1.2, X6=1.5, X8=2.0) is a setup-software current-loop slider — it
# equals the ODS "Max Torque" column on every row, which is the giveaway.
# Bench measurement on X6 #0 came in at 2.0095 (motor_id_protocol.html
# §10.2), 4.3 % below the 2.1 datasheet spec, well inside the ±5–10 %
# magnet/temperature spread. Match the existing scripts/current_monitor
# table.
#
# Stored as Decimal strings so 24.88 * 2.4 etc. don't bleed binary-FP
# noise into the YAML literals (factor_state for X8 would otherwise
# emit 0.059711999999999994 instead of 0.059712).
K_T = {"X4": Decimal("1.9"), "X6": Decimal("2.1"), "X8": Decimal("2.4")}

# I_rated — per-joint, from All_Updated_Motors_by_Type.ods (Rated Current
# column). Configured in drive flash via the MyActuator CAN setup tool;
# NOT readable over EtherCAT — every value here must be kept in sync with
# the physical drive's stored I_rated.
#
# right_shoulder_yaw_X4 reads 9.625 A in the ODS — confirmed typo, must be
# reconfigured to 8.625 A via CAN before this YAML's factor is correct.
# The dict reflects the post-fix state.
_X4_RATED = Decimal("8.625")
_X6_RATED = Decimal("13.40")
_X8_RATED = Decimal("24.88")
I_RATED = {
    # X4 — 10 joints, all 8.625 A
    "left_shoulder_yaw_X4":   _X4_RATED,
    "left_wrist_yaw_X4":      _X4_RATED,
    "left_wrist_roll_X4":     _X4_RATED,
    "left_ankle_pitch_X4":    _X4_RATED,
    "left_ankle_roll_X4":     _X4_RATED,
    "right_shoulder_yaw_X4":  _X4_RATED,
    "right_wrist_yaw_X4":     _X4_RATED,
    "right_wrist_roll_X4":    _X4_RATED,
    "right_ankle_pitch_X4":   _X4_RATED,
    "right_ankle_roll_X4":    _X4_RATED,
    # X6 — 6 joints, all 13.40 A
    "left_shoulder_pitch_X6":  _X6_RATED,
    "left_shoulder_roll_X6":   _X6_RATED,
    "left_elbow_pitch_X6":     _X6_RATED,
    "right_shoulder_pitch_X6": _X6_RATED,
    "right_shoulder_roll_X6":  _X6_RATED,
    "right_elbow_pitch_X6":    _X6_RATED,
    # X8 — 9 joints, all 24.88 A
    "waist_yaw_X8":         _X8_RATED,
    "left_hip_pitch_X8":    _X8_RATED,
    "left_hip_roll_X8":     _X8_RATED,
    "left_hip_yaw_X8":      _X8_RATED,
    "left_knee_X8":         _X8_RATED,
    "right_hip_pitch_X8":   _X8_RATED,
    "right_hip_roll_X8":    _X8_RATED,
    "right_hip_yaw_X8":     _X8_RATED,
    "right_knee_X8":        _X8_RATED,
}

# Expected pre-patch magnitudes — guard against partial migration.
LEGACY_FACTOR_CMD = 50.0
LEGACY_FACTOR_STATE = 0.02

# Slave-config trees that get patched in lockstep.
# parents[3] = .../src, parents[4] = workspace root.
SRC_DIR = Path(__file__).resolve().parents[3]
YAML_DIRS = [
    SRC_DIR / "bringup" / "arm_real_bringup" / "config" / "ethercat",
    SRC_DIR / "bringup" / "robot_bringup"    / "config" / "ethercat_pvt",
]
REPO_ROOT = SRC_DIR.parent


def family_of(joint_name: str) -> str:
    """Last `_X<n>` suffix names the motor family."""
    suffix = joint_name.rsplit("_", 1)[-1]
    if suffix not in K_T:
        raise ValueError(f"unknown motor family in {joint_name!r}")
    return suffix


def factors(family: str, i_rated: Decimal) -> tuple[float, float]:
    """Return (factor_cmd, factor_state) — no rounding, full Decimal precision
    converted to float at the very end so the YAML stores clean literals.

    factor_state is computed directly as rated_torque / 1000, not as
    1 / factor_cmd, to avoid the round-trip rounding error that otherwise
    leaks binary-FP noise (e.g. 0.059711999999999994 instead of 0.059712)."""
    rated_torque = K_T[family] * i_rated
    factor_cmd = Decimal(1000) / rated_torque
    factor_state = rated_torque / Decimal(1000)
    return float(factor_cmd), float(factor_state)


# ─── `print` subcommand ───────────────────────────────────────────────────────

def cmd_print() -> int:
    hdr = ("joint", "family", "I_rated[A]", "K_t[Nm/A]",
           "rated[Nm]", "factor_cmd[‰/Nm]", "factor_state[Nm/‰]")
    rows = []
    for joint, i_rated in I_RATED.items():
        fam = family_of(joint)
        f_cmd, f_state = factors(fam, i_rated)
        rows.append((joint, fam, f"{i_rated:.3f}", f"{K_T[fam]:.3f}",
                     f"{K_T[fam] * i_rated:.6f}",
                     f"{f_cmd:.6f}", f"{f_state:.6f}"))
    widths = [max(len(h), *(len(r[i]) for r in rows)) for i, h in enumerate(hdr)]
    fmt = "  ".join("{:<%d}" % w for w in widths)
    print(fmt.format(*hdr))
    print(fmt.format(*("-" * w for w in widths)))
    for r in rows:
        print(fmt.format(*r))
    return 0


# ─── `patch` subcommand ───────────────────────────────────────────────────────
#
# The patch updates only the two `factor: ...` literals in the effort-channel
# flow-style PDO entries, plus the matching stale "factor: ±50.0" /
# "factor: ±0.02" lines in the adjacent comment blocks. Targeted regex on
# the raw file text — NOT a YAML round-trip — because ruamel.yaml's emitter
# rewrites null shorthand (`~` → `null`), changes list-item indentation,
# and shifts inline comments, all of which would clobber the existing
# hand-tuned layout even when only one number changed.

_FACTOR_RE = re.compile(r"(factor:\s*)(-?\d+(?:\.\d+)?(?:e[+-]?\d+)?)")

# Stale comment lines we want to rewrite, keyed by the literal magnitude
# they reference. The replacement points at the formula and at this script
# so the comment never goes stale again when K_t or I_rated changes.
_COMMENT_REWRITES = {
    "#   factor: ±50.0 (sign matches motor direction)":
        "#   factor: 1000 / (K_t · I_rated) — per-family, "
        "see scripts/recompute_torque_factors.py; sign matches direction",
    "#   factor: ±0.02 converts to normalized fraction (sign matches direction)":
        "#   factor: K_t · I_rated / 1000 — per-family, "
        "see scripts/recompute_torque_factors.py; sign matches direction",
}


def _patch_line(line: str, want_role: str, expected_abs: float,
                new_abs: float, *, force: bool) -> tuple[str, str | None]:
    """If `line` is the effort PDO entry for `want_role`, return the rewritten
    line and a status message. Otherwise return the line unchanged and None.

    want_role is either "command_interface: effort" or
    "state_interface: effort". Sign is preserved from the existing value.
    The legacy-magnitude guard refuses to overwrite a value whose
    magnitude is not `expected_abs`, unless --force is set."""
    if want_role not in line:
        return line, None
    m = _FACTOR_RE.search(line)
    if m is None:
        return line, f"  WARN  no factor field on line: {line.strip()!r}"
    current = float(m.group(2))
    if current >= 0:
        new_value = new_abs
    else:
        new_value = -new_abs
    if current == new_value:
        return line, None  # already up to date
    if abs(current) != expected_abs and not force:
        return line, (f"  GUARD {want_role} factor is {current} "
                      f"(expected ±{expected_abs}); use --force to overwrite")
    new_text = repr(new_value)
    new_line = line[:m.start(2)] + new_text + line[m.end(2):]
    return new_line, f"  {want_role}: factor {current} -> {new_value}"


def patch_file(path: Path, *, apply: bool, force: bool) -> tuple[bool, list[str]]:
    """Patch one slave YAML. Returns (changed, messages)."""
    joint = path.stem
    msgs: list[str] = []
    if joint not in I_RATED:
        msgs.append(f"  SKIP  unknown joint {joint!r}")
        return False, msgs
    fam = family_of(joint)
    new_cmd_abs, new_state_abs = factors(fam, I_RATED[joint])

    text = path.read_text()
    out_lines = []
    changed = False
    for line in text.splitlines(keepends=True):
        # 1. Factor literals on the PDO entries.
        new_line, msg = _patch_line(
            line, "command_interface: effort",
            LEGACY_FACTOR_CMD, new_cmd_abs, force=force)
        if new_line is line:
            new_line, msg = _patch_line(
                line, "state_interface: effort",
                LEGACY_FACTOR_STATE, new_state_abs, force=force)
        # 2. Stale `factor: ±50.0` / `±0.02` lines in surrounding comments.
        if new_line is line and msg is None:
            stripped = line.strip()
            for stale, fresh in _COMMENT_REWRITES.items():
                if stripped == stale:
                    indent = line[:len(line) - len(line.lstrip())]
                    eol = line[len(line.rstrip("\r\n")):]
                    new_line = indent + fresh + eol
                    msg = "  comment rewritten"
                    break
        if new_line is not line:
            changed = True
        if msg is not None:
            msgs.append(msg)
        out_lines.append(new_line)

    if changed and apply:
        path.write_text("".join(out_lines))
    return changed, msgs


def cmd_patch(apply: bool, force: bool) -> int:
    any_change = False
    missing_dirs = [d for d in YAML_DIRS if not d.is_dir()]
    if missing_dirs:
        for d in missing_dirs:
            print(f"missing: {d}", file=sys.stderr)
        return 2

    for d in YAML_DIRS:
        print(f"\n=== {d.relative_to(REPO_ROOT)} ===")
        for path in sorted(d.glob("*.yaml")):
            changed, msgs = patch_file(path, apply=apply, force=force)
            if msgs:
                print(f"{path.name}")
                for m in msgs:
                    print(m)
            if changed:
                any_change = True

    if not any_change:
        print("\nno changes")
        return 0
    if apply:
        print("\nwrote changes")
    else:
        print("\nDRY RUN — re-run with `patch --apply` to write")
    return 0


# ─── CLI ──────────────────────────────────────────────────────────────────────

def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[1].strip(),
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("print", help="print the per-joint factor table")
    sp = sub.add_parser("patch", help="update factor literals in slave YAMLs")
    g = sp.add_mutually_exclusive_group(required=True)
    g.add_argument("--dry-run", action="store_true",
                   help="report what would change, don't write")
    g.add_argument("--apply", action="store_true", help="write changes")
    sp.add_argument("--force", action="store_true",
                    help="bypass the legacy-magnitude guard")
    args = p.parse_args(argv)

    if args.cmd == "print":
        return cmd_print()
    return cmd_patch(apply=args.apply, force=args.force)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
