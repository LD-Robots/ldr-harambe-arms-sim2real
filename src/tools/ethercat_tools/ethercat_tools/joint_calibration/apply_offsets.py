#!/usr/bin/env python3
"""Apply calibration offsets to EtherCAT slave YAML config files.

Reads a dict of {filename: {rpdo, tpdo}} offset values and patches the
corresponding rpdo position (command_interface: position) and tpdo position
(state_interface: position) offset fields in each YAML file.

Supports both layouts the ICube driver accepts:
  - Legacy CSP (arm_real_bringup/config/ethercat/):
      ..., factor: <f>, offset: <o>, default: <d>}     (RxPDO position)
      ..., factor: <f>, offset: <o>}                   (TxPDO position)
  - PVT mode 5 (robot_bringup/config/ethercat_pvt/):
      ..., factor: <f>}                                (no offset/default)

For the PVT layout, the offset field is *inserted* on first write; on
subsequent writes the same replace path used for legacy fires.

Can be used standalone (CLI) or imported by the calibration GUI.

Usage (standalone):
    python3 apply_offsets.py offsets.yaml                # auto-detect dir
    python3 apply_offsets.py --mode pvt offsets.yaml     # force PVT dir
    python3 apply_offsets.py --mode legacy offsets.yaml  # force legacy dir
    python3 apply_offsets.py --dry-run offsets.yaml      # preview only

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


_LEGACY_PKG = "arm_real_bringup"
_LEGACY_SUBDIR = os.path.join("config", "ethercat")
_PVT_PKG = "robot_bringup"
_PVT_SUBDIR = os.path.join("config", "ethercat_pvt")


def _source_tree_dir(pkg, subdir):
    """Return source-tree path src/bringup/<pkg>/<subdir> or None."""
    this_dir = os.path.dirname(os.path.realpath(__file__))
    ws_src = os.path.abspath(os.path.join(this_dir, "..", "..", "..", "..", ".."))
    d = os.path.join(ws_src, "src", "bringup", pkg, subdir)
    return d if os.path.isdir(d) else None


def _installed_dir(pkg, subdir):
    """Return ament install share path for <pkg>/<subdir> or None."""
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory(pkg), subdir)
        return d if os.path.isdir(d) else None
    except Exception:
        return None


def find_legacy_ethercat_dir():
    """Locate arm_real_bringup/config/ethercat/."""
    return _source_tree_dir(_LEGACY_PKG, _LEGACY_SUBDIR) \
        or _installed_dir(_LEGACY_PKG, _LEGACY_SUBDIR)


def find_pvt_ethercat_dir():
    """Locate robot_bringup/config/ethercat_pvt/."""
    return _source_tree_dir(_PVT_PKG, _PVT_SUBDIR) \
        or _installed_dir(_PVT_PKG, _PVT_SUBDIR)


def find_active_ethercat_dir(mode="auto"):
    """Locate the EtherCAT slave config directory.

    mode:
      "pvt"    — robot_bringup/config/ethercat_pvt/
      "legacy" — arm_real_bringup/config/ethercat/
      "auto"   — prefer PVT if present, else legacy
    """
    if mode == "pvt":
        return find_pvt_ethercat_dir()
    if mode == "legacy":
        return find_legacy_ethercat_dir()
    # auto: prefer PVT, fall back to legacy
    return find_pvt_ethercat_dir() or find_legacy_ethercat_dir()


# Back-compat alias — old callers expected the legacy path specifically.
def find_ethercat_dir():
    return find_legacy_ethercat_dir()


# Replace existing offset (legacy: factor: F, offset: O, default: D};
# also matches PVT files after the first insert: factor: F, offset: O}).
_RPDO_REPLACE = re.compile(
    r"(command_interface:\s*position,\s*factor:\s*[^,}]+,\s*offset:\s*)"
    r"[^,}]+"
    r"(\s*(?:,\s*default:\s*[^,}]+)?\s*\})"
)

# Insert offset for PVT layout (factor: F} with no offset field).
# Factor value is bounded by [^,}\s]+ so we stop before the closing brace
# without swallowing trailing whitespace.
_RPDO_INSERT = re.compile(
    r"(command_interface:\s*position,\s*factor:\s*[^,}\s]+)(\s*\})"
)

_TPDO_REPLACE = re.compile(
    r"(state_interface:\s*position,\s*factor:\s*[^,}]+,\s*offset:\s*)"
    r"[^,}]+"
    r"(\s*\})"
)

_TPDO_INSERT = re.compile(
    r"(state_interface:\s*position,\s*factor:\s*[^,}\s]+)(\s*\})"
)


def _patch_rpdo(content, rpdo_val):
    new_content, count = _RPDO_REPLACE.subn(
        lambda m: f"{m.group(1)}{int(rpdo_val)}{m.group(2)}",
        content,
    )
    if count == 0:
        new_content, count = _RPDO_INSERT.subn(
            lambda m: f"{m.group(1)}, offset: {int(rpdo_val)}{m.group(2)}",
            content,
        )
    return new_content, count


def _patch_tpdo(content, tpdo_val):
    new_content, count = _TPDO_REPLACE.subn(
        lambda m: f"{m.group(1)}{float(tpdo_val):.6f}{m.group(2)}",
        content,
    )
    if count == 0:
        new_content, count = _TPDO_INSERT.subn(
            lambda m: f"{m.group(1)}, offset: {float(tpdo_val):.6f}{m.group(2)}",
            content,
        )
    return new_content, count


def apply_offsets(offsets, ethercat_dir=None, dry_run=False, mode="auto"):
    """Patch EtherCAT YAML files with new calibration offsets.

    Args:
        offsets: dict of {filename: {"rpdo": int, "tpdo": float}}
        ethercat_dir: path to config dir (auto-detected from `mode` if None)
        dry_run: if True, print changes without writing
        mode: "auto", "pvt", or "legacy" (only used when ethercat_dir is None)

    Returns:
        list of (filename, success_bool, message) tuples
    """
    if ethercat_dir is None:
        ethercat_dir = find_active_ethercat_dir(mode=mode)
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

            new_content, rpdo_count = _patch_rpdo(content, rpdo_val)
            new_content, tpdo_count = _patch_tpdo(new_content, tpdo_val)

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
        "--mode", choices=("auto", "pvt", "legacy"), default="auto",
        help="Which config tree to target when --ethercat-dir is omitted "
             "(default: auto — prefers PVT if present)",
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

    results = apply_offsets(
        offsets,
        ethercat_dir=args.ethercat_dir,
        dry_run=args.dry_run,
        mode=args.mode,
    )

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
