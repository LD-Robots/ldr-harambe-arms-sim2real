#!/usr/bin/env python3
"""EtherCAT PDO factor & offset formulas for MyActuator RMD motors.

This module documents and implements every conversion formula used by the
ICube ethercat_driver_ros2 framework for position PDO channels, and provides
verification routines to validate existing per-joint YAML slave configs.

=== Encoder → Factor Derivation ===

    Each motor has an N-bit absolute output-shaft encoder.
    The drive firmware maps:  ±RAW_PER_PI counts = ±180° = ±π rad.

        RAW_PER_PI = 2^N / 2 = 2^(N-1)
            X6/X8 (17-bit): 2^16 = 65536
            X4    (18-bit): 2^17 = 131072

        state_factor = direction × π / RAW_PER_PI   [rad/count]
        cmd_factor   = direction × RAW_PER_PI / π   [counts/rad]

    These are exact inverses:  cmd_factor × state_factor = 1.0

=== ICube Driver Formula ===

    Source: ec_pdo_single_interface_channel_manager.cpp

    TxPDO read  (line 142):  state_value = factor × raw_value + offset
    RxPDO write (line 163):  raw_value   = factor × command   + offset

=== Offset Formulas ===

    Source: offset_window.py (demo_joint_offset calibration tool), lines 59-95

    Given offset_raw (the raw encoder displacement from URDF zero):

        offset_rpdo = direction × offset_raw          [raw counts]
        offset_tpdo = -(offset_raw × π / RAW_PER_PI)  [radians]

    Cross-check (eliminating offset_raw):

        offset_tpdo = -offset_rpdo × state_factor

=== Verification Checks ===

    1. Factor inverse:    cmd_factor × state_factor ≈ 1.0
    2. Offset cross:      offset_tpdo ≈ -offset_rpdo × state_factor
    3. Zero roundtrip:    cmd(0) → raw → read() ≈ 0.0
    4. Arbitrary roundtrip at ±1 rad and ±π/2 rad
"""

from __future__ import annotations

import math
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


# ═══════════════════════════════════════════════════════════════════════════════
# Constants
# ═══════════════════════════════════════════════════════════════════════════════

OUTPUT_ENCODER_BITS: dict[str, int] = {
    "X6": 17,
    "X8": 17,
    "X4": 18,
}

RAW_PER_PI: dict[str, int] = {
    motor: (1 << bits) // 2
    for motor, bits in OUTPUT_ENCODER_BITS.items()
}
"""Encoder counts per π radians (180°) for each motor type.

    X6/X8 (17-bit): 2^16 = 65536 counts/π
    X4    (18-bit): 2^17 = 131072 counts/π
"""


# ═══════════════════════════════════════════════════════════════════════════════
# Factor formulas
# ═══════════════════════════════════════════════════════════════════════════════

def derive_raw_per_pi(encoder_bits: int) -> int:
    """Derive RAW_PER_PI from output encoder bit depth.

    RAW_PER_PI = 2^N / 2 = 2^(N-1)

    Args:
        encoder_bits: Output encoder resolution in bits (e.g. 17, 18).

    Returns:
        Encoder counts per π radians.
    """
    return (1 << encoder_bits) // 2


def compute_state_factor(motor: str, direction: int) -> float:
    """TxPDO position factor: raw encoder counts → radians.

    state_factor = direction × π / RAW_PER_PI

    Args:
        motor: Motor type ("X4", "X6", "X8").
        direction: +1 or -1.

    Returns:
        Factor in rad/count.
    """
    if direction not in (1, -1):
        raise ValueError(f"direction must be +1 or -1, got {direction}")
    return direction * math.pi / RAW_PER_PI[motor]


def compute_cmd_factor(motor: str, direction: int) -> float:
    """RxPDO position factor: radians → raw encoder counts.

    cmd_factor = direction × RAW_PER_PI / π

    Args:
        motor: Motor type ("X4", "X6", "X8").
        direction: +1 or -1.

    Returns:
        Factor in counts/rad.
    """
    if direction not in (1, -1):
        raise ValueError(f"direction must be +1 or -1, got {direction}")
    return direction * RAW_PER_PI[motor] / math.pi


# ═══════════════════════════════════════════════════════════════════════════════
# Offset formulas
# ═══════════════════════════════════════════════════════════════════════════════

def offset_rpdo_from_raw(offset_raw: int, direction: int) -> int:
    """Compute RxPDO position offset (raw counts) from calibration offset_raw.

    offset_rpdo = direction × offset_raw

    Args:
        offset_raw: Raw encoder displacement from URDF zero (direction-neutral).
        direction: +1 or -1.

    Returns:
        RxPDO offset in raw encoder counts.
    """
    return direction * offset_raw


def offset_tpdo_from_raw(offset_raw: int, motor: str) -> float:
    """Compute TxPDO position offset (radians) from calibration offset_raw.

    offset_tpdo = -(offset_raw × π / RAW_PER_PI)

    Args:
        offset_raw: Raw encoder displacement from URDF zero (direction-neutral).
        motor: Motor type ("X4", "X6", "X8").

    Returns:
        TxPDO offset in radians.
    """
    return -(offset_raw * math.pi / RAW_PER_PI[motor])


def offset_tpdo_from_rpdo(offset_rpdo: float, state_factor: float) -> float:
    """Cross-check: derive TxPDO offset directly from RxPDO offset.

    offset_tpdo = -offset_rpdo × state_factor

    Args:
        offset_rpdo: RxPDO position offset (raw counts).
        state_factor: TxPDO position factor (rad/count), including direction sign.

    Returns:
        Expected TxPDO offset in radians.
    """
    return -offset_rpdo * state_factor


# ═══════════════════════════════════════════════════════════════════════════════
# ICube driver simulation
# ═══════════════════════════════════════════════════════════════════════════════

def icube_tpdo_read(raw: float, factor: float, offset: float) -> float:
    """Simulate ICube driver TxPDO read (ec_read, line 142).

    state_value = factor × raw + offset
    """
    return factor * raw + offset


def icube_rpdo_write(command: float, factor: float, offset: float) -> float:
    """Simulate ICube driver RxPDO write (ec_write, line 163).

    raw = factor × command + offset
    """
    return factor * command + offset


# ═══════════════════════════════════════════════════════════════════════════════
# Verification
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class CheckResult:
    """Result of a single verification check."""
    name: str
    passed: bool
    expected: float
    actual: float
    tolerance: float
    detail: str = ""

    def __str__(self) -> str:
        status = "PASS" if self.passed else "FAIL"
        return (
            f"  {status}: {self.name:<28s}  "
            f"expected={self.expected:+.8f}  "
            f"actual={self.actual:+.8f}  "
            f"err={abs(self.expected - self.actual):.2e}"
            f"{'  ' + self.detail if self.detail else ''}"
        )


def _check(
    name: str,
    expected: float,
    actual: float,
    tol: float,
    detail: str = "",
) -> CheckResult:
    passed = abs(expected - actual) <= tol
    return CheckResult(name, passed, expected, actual, tol, detail)


def verify_factor_pair(
    cmd_factor: float,
    state_factor: float,
    tol: float = 1e-3,
) -> CheckResult:
    """Check that cmd_factor × state_factor ≈ 1.0.

    Tolerance is 1e-3 because YAML configs truncate factor values
    (e.g. 20861.0 vs exact 65535/π = 20860.84...), giving ~7.6e-5 error.
    """
    product = cmd_factor * state_factor
    return _check("factor_inverse", 1.0, product, tol)


def verify_offset_pair(
    offset_rpdo: float,
    offset_tpdo: float,
    state_factor: float,
    tol: float = 5e-3,
) -> CheckResult:
    """Check that offset_tpdo ≈ -offset_rpdo × state_factor."""
    expected = -offset_rpdo * state_factor
    return _check("offset_cross", expected, offset_tpdo, tol)


def verify_zero_roundtrip(
    cmd_factor: float,
    offset_rpdo: float,
    state_factor: float,
    offset_tpdo: float,
    tol: float = 5e-3,
) -> CheckResult:
    """Command 0 rad → raw → read back should ≈ 0 rad.

    raw = cmd_factor × 0 + offset_rpdo = offset_rpdo
    state = state_factor × offset_rpdo + offset_tpdo
    """
    raw = icube_rpdo_write(0.0, cmd_factor, offset_rpdo)
    state = icube_tpdo_read(raw, state_factor, offset_tpdo)
    return _check("zero_roundtrip", 0.0, state, tol)


def verify_arbitrary_roundtrip(
    cmd_factor: float,
    offset_rpdo: float,
    state_factor: float,
    offset_tpdo: float,
    command_rad: float,
    tol: float = 5e-3,
) -> CheckResult:
    """Command → raw → read back should recover the original command.

    raw = cmd_factor × command + offset_rpdo
    state = state_factor × raw + offset_tpdo
    """
    raw = icube_rpdo_write(command_rad, cmd_factor, offset_rpdo)
    state = icube_tpdo_read(raw, state_factor, offset_tpdo)
    return _check(
        f"roundtrip({command_rad:+.4f} rad)",
        command_rad,
        state,
        tol,
    )


def verify_all(
    cmd_factor: float,
    state_factor: float,
    offset_rpdo: float,
    offset_tpdo: float,
) -> list[CheckResult]:
    """Run the complete verification suite on one joint's parameters.

    Checks performed:
        1. Factor inverse:       cmd_factor × state_factor ≈ 1.0
        2. Offset cross-check:   offset_tpdo ≈ -offset_rpdo × state_factor
        3. Zero roundtrip:       cmd(0) → raw → read ≈ 0.0
        4. Roundtrip at +1.0 rad
        5. Roundtrip at -1.0 rad
        6. Roundtrip at +π/2 rad
        7. Roundtrip at -π/2 rad

    Returns:
        List of CheckResult (one per check).
    """
    results = [
        verify_factor_pair(cmd_factor, state_factor),
        verify_offset_pair(offset_rpdo, offset_tpdo, state_factor),
        verify_zero_roundtrip(cmd_factor, offset_rpdo, state_factor, offset_tpdo),
    ]
    for rad in [1.0, -1.0, math.pi / 2, -math.pi / 2]:
        results.append(
            verify_arbitrary_roundtrip(
                cmd_factor, offset_rpdo, state_factor, offset_tpdo, rad,
            )
        )
    return results


# ═══════════════════════════════════════════════════════════════════════════════
# YAML config parser
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class PositionPdoParams:
    """Position PDO parameters extracted from a slave YAML config."""
    filename: str
    cmd_factor: float
    cmd_offset: float   # offset_rpdo (raw counts)
    state_factor: float
    state_offset: float  # offset_tpdo (radians)

    @property
    def direction(self) -> int:
        return 1 if self.cmd_factor > 0 else -1

    @property
    def motor(self) -> Optional[str]:
        """Detect motor type from cmd_factor magnitude.

        YAML configs use rounded values (20861.0 for X6, 41722.0 for X4),
        so we match with a tolerance of 5.0 counts/rad.
        """
        abs_cmd = abs(self.cmd_factor)
        for motor, rpp in RAW_PER_PI.items():
            expected = rpp / math.pi
            if abs(abs_cmd - expected) < 5.0:
                return motor
        return None


# Regex patterns for extracting position factor/offset from YAML lines.
# Matches: command_interface: position, factor: <F>, offset: <O>, default: ...
_RPDO_RE = re.compile(
    r"command_interface:\s*position\s*,"
    r"\s*factor:\s*(?P<factor>[^,]+)\s*,"
    r"\s*offset:\s*(?P<offset>[^,]+)\s*,"
)

# Matches: state_interface: position, factor: <F>, offset: <O>}
_TPDO_RE = re.compile(
    r"state_interface:\s*position\s*,"
    r"\s*factor:\s*(?P<factor>[^,}]+)\s*,"
    r"\s*offset:\s*(?P<offset>[^}]+)\s*\}"
)


def parse_yaml_position_params(yaml_path: str) -> Optional[PositionPdoParams]:
    """Extract position PDO factors and offsets from a slave YAML config.

    Uses regex (not full YAML parse) to match the exact line format used in
    arm_real_bringup/config/ethercat/ files. Returns None if either the
    RxPDO or TxPDO position channel is not found.
    """
    with open(yaml_path, "r") as f:
        content = f.read()

    rpdo_match = _RPDO_RE.search(content)
    tpdo_match = _TPDO_RE.search(content)

    if rpdo_match is None or tpdo_match is None:
        return None

    return PositionPdoParams(
        filename=os.path.basename(yaml_path),
        cmd_factor=float(rpdo_match.group("factor").strip()),
        cmd_offset=float(rpdo_match.group("offset").strip()),
        state_factor=float(tpdo_match.group("factor").strip()),
        state_offset=float(tpdo_match.group("offset").strip()),
    )


def verify_config_file(yaml_path: str) -> tuple[PositionPdoParams, list[CheckResult]]:
    """Parse a slave YAML and run full verification.

    Returns:
        Tuple of (parsed params, list of check results).

    Raises:
        ValueError: If the YAML file cannot be parsed.
    """
    params = parse_yaml_position_params(yaml_path)
    if params is None:
        raise ValueError(f"Could not parse position PDO params from {yaml_path}")

    results = verify_all(
        params.cmd_factor,
        params.state_factor,
        params.cmd_offset,
        params.state_offset,
    )
    return params, results


# ═══════════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════════

def _find_ethercat_dir() -> Optional[str]:
    """Locate arm_real_bringup/config/ethercat/ from source tree."""
    here = Path(__file__).resolve().parent
    # Walk up to workspace root (5 levels: ethercat_tools/ethercat_tools/→tools/→src/→ws)
    ws = here
    for _ in range(6):
        candidate = ws / "src" / "bringup" / "arm_real_bringup" / "config" / "ethercat"
        if candidate.is_dir():
            return str(candidate)
        ws = ws.parent
    return None


def main() -> int:
    """Scan all slave YAML configs and run verification suite."""
    ethercat_dir = _find_ethercat_dir()
    if ethercat_dir is None:
        print("ERROR: Could not find arm_real_bringup/config/ethercat/ directory.")
        return 1

    # Collect YAML files (arm joints only: X4 and X6)
    yaml_files = sorted(
        p for p in Path(ethercat_dir).glob("*_X[468].yaml")
    )

    if not yaml_files:
        print(f"No *_X[468].yaml files found in {ethercat_dir}")
        return 1

    total_pass = 0
    total_fail = 0
    failed_joints: list[str] = []

    print("=" * 90)
    print("EtherCAT PDO Formula Verification")
    print(f"Config dir: {ethercat_dir}")
    print(f"Files: {len(yaml_files)}")
    print("=" * 90)

    for yaml_path in yaml_files:
        fname = yaml_path.name
        try:
            params, results = verify_config_file(str(yaml_path))
        except ValueError as e:
            print(f"\n{fname}: SKIP — {e}")
            continue

        motor = params.motor or "???"
        joint_pass = sum(1 for r in results if r.passed)
        joint_fail = sum(1 for r in results if not r.passed)
        total_pass += joint_pass
        total_fail += joint_fail

        status = "ALL PASS" if joint_fail == 0 else f"{joint_fail} FAILED"
        print(f"\n{fname}  (motor={motor}, dir={params.direction:+d})")
        print(f"  cmd_factor={params.cmd_factor:+.1f}  "
              f"state_factor={params.state_factor:+.6e}  "
              f"rpdo_offset={params.cmd_offset:+.0f}  "
              f"tpdo_offset={params.state_offset:+.6f}")
        print(f"  [{status}]  ({joint_pass}/{joint_pass + joint_fail} checks)")

        for r in results:
            if not r.passed:
                print(r)

        if joint_fail > 0:
            failed_joints.append(fname)

    # Summary
    print("\n" + "=" * 90)
    print(f"TOTAL: {total_pass} passed, {total_fail} failed "
          f"across {len(yaml_files)} config files")
    if failed_joints:
        print(f"FAILED joints: {', '.join(failed_joints)}")
    else:
        print("All joints verified successfully.")
    print("=" * 90)

    return 0 if total_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
