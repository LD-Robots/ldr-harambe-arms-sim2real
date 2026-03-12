#!/usr/bin/env python3
"""Apply calibration offsets to EtherCAT slave YAML config files.

Reads a dict of {filename: {rpdo, tpdo}} offset values and patches the
corresponding rpdo position (command_interface: position) and tpdo position
(state_interface: position) offset fields in each YAML file.

Can be used standalone (CLI) or imported by the calibration GUI.

Usage (standalone):
    python3 apply_offsets.py offsets.yaml          # from exported YAML
    python3 apply_offsets.py --dry-run offsets.yaml # preview without writing

The input YAML file format:
    right_shoulder_pitch_X6.yaml:
      rpdo: -56738
      tpdo: 2.719885
    waist_yaw_X8.yaml:
      rpdo: -53826
      tpdo: 2.580291
"""

import argparse
import os
import re
import sys

import yaml


def find_ethercat_dir():
    """Locate arm_real_bringup/config/ethercat/ directory.

    Tries source tree relative to this file first, then ament_index.
    """
    # Relative to source tree
    this_dir = os.path.dirname(os.path.abspath(__file__))
    ws_src = os.path.abspath(os.path.join(this_dir, "..", "..", "..", "..", ".."))
    ethercat_dir = os.path.join(
        ws_src, "src", "bringup", "arm_real_bringup", "config", "ethercat"
    )
    if os.path.isdir(ethercat_dir):
        return ethercat_dir

    # Installed workspace via ament_index
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_dir = get_package_share_directory("arm_real_bringup")
        ethercat_dir = os.path.join(pkg_dir, "config", "ethercat")
        if os.path.isdir(ethercat_dir):
            return ethercat_dir
    except Exception:
        pass

    return None


def apply_offsets(offsets, ethercat_dir=None, dry_run=False):
    """Patch EtherCAT YAML files with new calibration offsets.

    Args:
        offsets: dict of {filename: {"rpdo": int, "tpdo": float}}
        ethercat_dir: path to config/ethercat/ directory (auto-detected if None)
        dry_run: if True, print changes without writing

    Returns:
        list of (filename, success_bool, message) tuples
    """
    if ethercat_dir is None:
        ethercat_dir = find_ethercat_dir()
    if ethercat_dir is None:
        return [("*", False, "Could not locate ethercat config directory")]

    results = []

    for fname, vals in sorted(offsets.items()):
        fpath = os.path.join(ethercat_dir, fname)
        if not os.path.isfile(fpath):
            results.append((fname, False, "File not found"))
            continue

        rpdo_val = vals.get("rpdo")
        tpdo_val = vals.get("tpdo")
        if rpdo_val is None or tpdo_val is None:
            results.append((fname, False, "Missing rpdo or tpdo value"))
            continue

        try:
            with open(fpath, "r") as f:
                content = f.read()

            # Patch rpdo position offset (command_interface: position line)
            # Pattern: ...offset: <old>, default:...
            new_content, rpdo_count = re.subn(
                r"(command_interface: position,\s*factor:\s*[^,]+,\s*offset:\s*)"
                r"[^,]+(,\s*default:)",
                r"\g<1>" + str(int(rpdo_val)) + r"\2",
                content,
            )

            # Patch tpdo position offset (state_interface: position line)
            # Pattern: ...offset: <old>}
            new_content, tpdo_count = re.subn(
                r"(state_interface: position,\s*factor:\s*[^,}]+,\s*offset:\s*)"
                r"[^}]+(})",
                r"\g<1>" + f"{float(tpdo_val):.6f}" + r"\2",
                new_content,
            )

            if rpdo_count != 1 or tpdo_count != 1:
                results.append((
                    fname, False,
                    f"Unexpected match count: rpdo={rpdo_count}, tpdo={tpdo_count}"
                ))
                continue

            if dry_run:
                results.append((
                    fname, True,
                    f"[dry-run] rpdo={int(rpdo_val)}, tpdo={float(tpdo_val):.6f}"
                ))
            else:
                with open(fpath, "w") as f:
                    f.write(new_content)
                results.append((
                    fname, True,
                    f"rpdo={int(rpdo_val)}, tpdo={float(tpdo_val):.6f}"
                ))

        except Exception as e:
            results.append((fname, False, str(e)))

    return results


def main():
    parser = argparse.ArgumentParser(
        description="Apply calibration offsets to EtherCAT slave YAML configs"
    )
    parser.add_argument(
        "offsets_file",
        help="YAML file with offsets: {filename: {rpdo: int, tpdo: float}}",
    )
    parser.add_argument(
        "--ethercat-dir",
        help="Path to ethercat config directory (auto-detected if omitted)",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Preview changes without writing files",
    )
    args = parser.parse_args()

    with open(args.offsets_file, "r") as f:
        offsets = yaml.safe_load(f)

    if not isinstance(offsets, dict):
        print(f"Error: expected dict in {args.offsets_file}, got {type(offsets).__name__}")
        sys.exit(1)

    results = apply_offsets(offsets, ethercat_dir=args.ethercat_dir, dry_run=args.dry_run)

    ok = 0
    fail = 0
    for fname, success, msg in results:
        status = "OK" if success else "FAIL"
        print(f"  {status}: {fname} — {msg}")
        if success:
            ok += 1
        else:
            fail += 1

    print(f"\n{'[DRY RUN] ' if args.dry_run else ''}Updated {ok} files, {fail} failures.")
    sys.exit(0 if fail == 0 else 1)


if __name__ == "__main__":
    main()
