#!/usr/bin/env python3
# Generate ethercat_pvt/*.yaml from the canonical mode-8 CSP YAMLs in
# arm_real_bringup/config/ethercat/. Per-joint physical values (position /
# velocity / torque factors, calibrated offsets, max_torque) are read from each
# canonical file and re-emitted in the PVT (CiA 402 mode 5) layout described in
# docs/ldr-harambe-docs/hardware/ethercat_control_guide.html §15 (RxPDO 0x1601,
# 22 B) and §16 (TxPDO 0x1A02, 22 B).
#
# What changes vs canonical:
#   * mode_of_operation: 8  -> 5                 (CSP -> PVT)
#   * interpolation period: 10 ms -> 1 ms        (100 Hz -> 1 kHz)
#   * RxPDO 0x1600 -> 0x1601                     (adds 0x2000 PVT_KP, 0x2001 PVT_KD)
#   * TxPDO 0x1A00 -> 0x1A02                     (adds 0x2009 motor_temp, 0x200C drive_temp, 0x200A bus_voltage)
#   * mode_of_operation channel: command_interface mode_of_operation, default 8
#     becomes command_interface ~ default 5      (PVT-only; the MyActuator firmware
#                                                  doesn't support remapping PDOs at runtime,
#                                                  so this PDO is locked to mode 5)
#   * 0x6072 max_torque RxPDO channel: removed   (not part of 0x1601 layout; SDO still applies)
#   * Padding bytes 0x5FF1/0x5FF2 -> 0x2FFD/0x2FFE per firmware ESI MT-Device_260424.xml
#
# What is preserved:
#   * vendor_id / product_id / assign_activate
#   * auto_fault_reset / auto_state_transitions
#   * SDO max_torque value (per-joint, e.g. 800 for legs, 500 for shoulders, 400 for wrists)
#   * Position / velocity / torque factors and signs (direction reversal stays correct)
#   * Calibrated position offsets

import argparse
import os
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.stderr.write("generate_ethercat_pvt: PyYAML not available\n")
    sys.exit(1)


def _find_channel(channels, index):
    for ch in channels:
        if ch.get("index") == index:
            return ch
    raise KeyError(f"channel 0x{index:04x} not found in canonical YAML")


def _find_sdo(sdos, index, sub_index):
    for s in sdos:
        if s.get("index") == index and s.get("sub_index") == sub_index:
            return s
    raise KeyError(f"SDO 0x{index:04x}:{sub_index} not found")


def _fmt(value):
    """Format a numeric value preserving the canonical precision.

    PyYAML loads ``-20860.756`` as the float -20860.756; ``repr`` round-trips
    cleanly for these values. Integers stay integers. Per memory
    [[feedback-no-rounding]] we never truncate.
    """
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return repr(value)
    return str(value)


def _fmt_hex(value, width):
    """Format an integer as a 0x-prefixed hex literal of fixed width.

    Used for vendor_id / product_id / assign_activate which PyYAML loads as
    plain ints but readers expect to see in hex form.
    """
    return f"0x{int(value):0{width}X}"


def generate_pvt_yaml(canonical_path: Path) -> str:
    with canonical_path.open() as f:
        data = yaml.safe_load(f)

    joint_name = canonical_path.stem  # e.g. "left_shoulder_pitch_X6"

    vendor_id = data["vendor_id"]
    product_id = data["product_id"]
    assign_activate = data["assign_activate"]
    auto_fault_reset = data["auto_fault_reset"]
    auto_state_transitions = data["auto_state_transitions"]

    # Per-joint max_torque from canonical SDO (preserve the existing per-joint limit).
    max_torque = _find_sdo(data["sdo"], 0x6072, 0)["value"]

    rpdo_channels = data["rpdo"][0]["channels"]
    tpdo_channels = data["tpdo"][0]["channels"]

    # Pull factors / offsets from canonical RxPDO (0x1600).
    pos_cmd = _find_channel(rpdo_channels, 0x607A)
    vel_cmd = _find_channel(rpdo_channels, 0x60FF)
    eff_cmd = _find_channel(rpdo_channels, 0x6071)

    pos_cmd_factor = pos_cmd["factor"]
    pos_cmd_offset = pos_cmd.get("offset", 0)
    pos_cmd_default = pos_cmd.get("default", pos_cmd_offset)
    vel_cmd_factor = vel_cmd["factor"]
    eff_cmd_factor = eff_cmd["factor"]

    # Pull factors / offsets from canonical TxPDO (0x1A00).
    pos_state = _find_channel(tpdo_channels, 0x6064)
    vel_state = _find_channel(tpdo_channels, 0x606C)
    eff_state = _find_channel(tpdo_channels, 0x6077)

    pos_state_factor = pos_state["factor"]
    pos_state_offset = pos_state.get("offset", 0)
    vel_state_factor = vel_state["factor"]
    eff_state_factor = eff_state["factor"]

    # ── Render ────────────────────────────────────────────────────────────────
    # Hand-templated YAML so we keep inline channel style + comments. PyYAML's
    # default dumper would expand each channel to a multi-line block, which is
    # noisier and harder to diff against the canonical mode-8 file.
    out = []
    out.append("# ═══════════════════════════════════════════════════════════════════════════════")
    out.append(f"# ICube EtherCAT slave config — {joint_name} (PVT / CiA 402 mode 5)")
    out.append("#")
    out.append("# BOOTSTRAPPED from arm_real_bringup/config/ethercat/{name}.yaml via")
    out.append("# scripts/generate_ethercat_pvt.py. After bootstrapping this is the")
    out.append("# source of truth — edit by hand for per-joint PVT tuning (PVT_KP / PVT_KD")
    out.append("# factors, max_torque, position offsets, etc.). Re-running the generator")
    out.append("# will OVERWRITE these edits.")
    out.append("#")
    out.append("# Mode 5 (PVT) layout:")
    out.append("#   RxPDO 0x1601 (22 B): control_word, target_position, target_velocity,")
    out.append("#                        torque_ff, PVT_KP, PVT_KD, mode (locked 5), padding.")
    out.append("#   TxPDO 0x1A02 (22 B): status, actual_pos, actual_vel, actual_torque,")
    out.append("#                        error_code, motor_temp, drive_temp, bus_voltage,")
    out.append("#                        mode_display, padding.")
    out.append("#")
    out.append("# Drive math:  τ = Kp·(p_des - p_fb) + Kd·(v_des - v_fb) + τ_ff")
    out.append("#   PVT_KP / PVT_KD apply ×0.001 inside the drive, so factor=1000.0 lets")
    out.append("#   the host command in N·m/rad and N·m·s/rad.")
    out.append("# ═══════════════════════════════════════════════════════════════════════════════")
    out.append("")
    out.append("# ─── DEVICE IDENTITY ───")
    out.append(f"vendor_id:  {_fmt_hex(vendor_id, 8)}")
    out.append(f"product_id: {_fmt_hex(product_id, 8)}")
    out.append("")
    out.append("# ─── DISTRIBUTED CLOCK ───")
    out.append(f"assign_activate: {_fmt_hex(assign_activate, 4)}")
    out.append("")
    out.append("# ─── CiA 402 DRIVE BEHAVIOR ───")
    out.append(f"auto_fault_reset: {_fmt(auto_fault_reset)}")
    out.append(f"auto_state_transitions: {_fmt(auto_state_transitions)}")
    out.append("mode_of_operation: 5            # 5 = PVT (drive runs Kp·Δp + Kd·Δv + τ_ff)")
    out.append("")
    out.append("# ─── SDO: one-time parameters written to drive at startup ───")
    out.append("sdo:")
    out.append(f"  - {{index: 0x6072, sub_index: 0, type: uint16, value: {_fmt(max_torque)}}}  # max torque (per-mille rated)")
    out.append("  - {index: 0x60C2, sub_index: 1, type: int8,   value: 1}     # interpolation time period")
    out.append("  - {index: 0x60C2, sub_index: 2, type: int8,   value: -3}    # base 10^-3 s → 1 ms cycle = 1 kHz")
    out.append("")
    out.append("# ─── RxPDO 0x1601 (22 B): master → drive every cycle ───")
    out.append("rpdo:")
    out.append("  - index: 0x1601")
    out.append("    channels:")
    out.append("      # [2 B] control_word — managed by CiA 402 driver")
    out.append("      - {index: 0x6040, sub_index: 0, type: uint16, command_interface: ~,        default: 0}")
    out.append("      # [4 B] target_position (0x607A) — drive PD setpoint, in encoder counts (factor includes direction sign)")
    out.append(f"      - {{index: 0x607a, sub_index: 0, type: int32,  command_interface: position, factor: {_fmt(pos_cmd_factor)}, offset: {_fmt(pos_cmd_offset)}, default: {_fmt(pos_cmd_default)}}}")
    out.append("      # [4 B] target_velocity (0x60FF) — drive PD velocity setpoint")
    out.append(f"      - {{index: 0x60ff, sub_index: 0, type: int32,  command_interface: velocity, factor: {_fmt(vel_cmd_factor)}, default: 0}}")
    out.append("      # [2 B] torque feedforward (0x6071) — added to PD output inside the drive")
    out.append(f"      - {{index: 0x6071, sub_index: 0, type: int16,  command_interface: effort,   factor: {_fmt(eff_cmd_factor)},    default: 0}}")
    out.append("      # [4 B] PVT_KP (0x2000) — drive applies value × 0.001; factor 1000 ⇒ host sends N·m/rad")
    out.append("      - {index: 0x2000, sub_index: 0, type: int32,  command_interface: kp,       factor: 1000.0, default: 0}")
    out.append("      # [4 B] PVT_KD (0x2001) — same scaling as Kp")
    out.append("      - {index: 0x2001, sub_index: 0, type: int32,  command_interface: kd,       factor: 1000.0, default: 0}")
    out.append("      # [1 B] mode_of_operation (0x6060) — locked to PVT (firmware does not remap PDOs)")
    out.append("      - {index: 0x6060, sub_index: 0, type: int8,   command_interface: ~,        default: 5}")
    out.append("      # [1 B] padding")
    out.append("      - {index: 0x2ffd, sub_index: 0, type: int8,   command_interface: ~,        default: 0}")
    out.append("")
    out.append("# ─── TxPDO 0x1A02 (22 B): drive → master every cycle ───")
    out.append("# Channel order must match the ESI MT-Device_260424.xml; this firmware does not")
    out.append("# remap TxPDOs, so reordering will cause AL status 0x001E during PREOP→SAFEOP.")
    out.append("tpdo:")
    out.append("  - index: 0x1a02")
    out.append("    channels:")
    out.append("      # [2 B] status_word (0x6041) — exposed for safety supervisor + diagnostics")
    out.append("      - {index: 0x6041, sub_index: 0, type: uint16, state_interface: status_word}")
    out.append("      # [4 B] actual_position (0x6064)")
    out.append(f"      - {{index: 0x6064, sub_index: 0, type: int32,  state_interface: position, factor: {_fmt(pos_state_factor)}, offset: {_fmt(pos_state_offset)}}}")
    out.append("      # [4 B] actual_velocity (0x606C)")
    out.append(f"      - {{index: 0x606c, sub_index: 0, type: int32,  state_interface: velocity, factor: {_fmt(vel_state_factor)}}}")
    out.append("      # [2 B] actual_torque (0x6077)")
    out.append(f"      - {{index: 0x6077, sub_index: 0, type: int16,  state_interface: effort,   factor: {_fmt(eff_state_factor)}}}")
    out.append("      # [2 B] error_code (0x603F) — exposed to robot_drive_status_broadcaster")
    out.append("      - {index: 0x603f, sub_index: 0, type: uint16, state_interface: error_code}")
    out.append("      # [2 B] motor_temperature (0x2009) — degrees C, int16")
    out.append("      - {index: 0x2009, sub_index: 0, type: int16,  state_interface: motor_temperature, factor: 1.0}")
    out.append("      # [2 B] drive_temperature (0x200C) — MOSFET temp, degrees C, int16")
    out.append("      - {index: 0x200c, sub_index: 0, type: int16,  state_interface: drive_temperature, factor: 1.0}")
    out.append("      # [2 B] bus_voltage (0x200A) — V, rounded uint16")
    out.append("      - {index: 0x200a, sub_index: 0, type: uint16, state_interface: bus_voltage,       factor: 1.0}")
    out.append("      # [1 B] mode_of_operation_display (0x6061) — read by CiA 402 plugin")
    out.append("      - {index: 0x6061, sub_index: 0, type: int8,   state_interface: ~}")
    out.append("      # [1 B] padding")
    out.append("      - {index: 0x2ffe, sub_index: 0, type: int8,   state_interface: ~}")
    out.append("")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input-dir", required=True, type=Path)
    ap.add_argument("--output-dir", required=True, type=Path)
    args = ap.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)

    yamls = sorted(args.input_dir.glob("*.yaml"))
    if not yamls:
        sys.stderr.write(f"generate_ethercat_pvt: no YAMLs found in {args.input_dir}\n")
        sys.exit(1)

    for path in yamls:
        try:
            content = generate_pvt_yaml(path)
        except KeyError as e:
            sys.stderr.write(f"generate_ethercat_pvt: {path.name}: {e}\n")
            sys.exit(1)

        out_path = args.output_dir / path.name
        out_path.write_text(content)
        print(f"  PVT: {path.name}")

    print(f"generate_ethercat_pvt: emitted {len(yamls)} files to {args.output_dir}")


if __name__ == "__main__":
    main()
