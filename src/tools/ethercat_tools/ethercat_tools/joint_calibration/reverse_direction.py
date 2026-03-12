#!/usr/bin/env python3
"""Reverse motor direction for EtherCAT slave YAML config files.

Negates ALL factor values (position, velocity, effort — both rpdo and tpdo)
and updates the Direction comment header. This is equivalent to physically
reversing the motor rotation direction.

Can be used standalone (CLI) or imported by the calibration GUI.

Usage (standalone):
    python3 reverse_direction.py right_shoulder_roll_X6.yaml right_shoulder_yaw_X4.yaml
    python3 reverse_direction.py --dry-run right_wrist_yaw_X4.yaml
    python3 reverse_direction.py --list  # show current direction of all joints

After reversing, offsets must be recalibrated for the affected joints.
"""

import argparse
import os
import re
import sys


def find_ethercat_dirs():
    """Locate both ethercat/ and ethercat_readonly/ config directories.

    Returns list of existing directory paths.
    """
    dirs = []

    # Relative to source tree
    this_dir = os.path.dirname(os.path.abspath(__file__))
    ws_src = os.path.abspath(os.path.join(this_dir, "..", "..", "..", "..", ".."))
    for subdir in ("ethercat", "ethercat_readonly"):
        d = os.path.join(ws_src, "src", "bringup", "arm_real_bringup", "config", subdir)
        if os.path.isdir(d):
            dirs.append(d)

    if dirs:
        return dirs

    # Installed workspace via ament_index
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_dir = get_package_share_directory("arm_real_bringup")
        for subdir in ("ethercat", "ethercat_readonly"):
            d = os.path.join(pkg_dir, "config", subdir)
            if os.path.isdir(d):
                dirs.append(d)
    except Exception:
        pass

    return dirs


def _negate_factor(match):
    """Regex replacement: negate a factor value."""
    val = match.group(1)
    if val.startswith("-"):
        return f"factor: {val[1:]}"
    else:
        return f"factor: -{val}"


def reverse_direction(filenames, ethercat_dirs=None, dry_run=False):
    """Negate all factors in the given EtherCAT YAML config files.

    Args:
        filenames: list of YAML filenames (e.g. ["right_shoulder_roll_X6.yaml"])
        ethercat_dirs: list of config directory paths (auto-detected if None)
        dry_run: if True, print changes without writing

    Returns:
        list of (filename, directory, success_bool, message) tuples
    """
    if ethercat_dirs is None:
        ethercat_dirs = find_ethercat_dirs()
    if not ethercat_dirs:
        return [("*", "", False, "Could not locate ethercat config directories")]

    results = []

    for fname in filenames:
        for d in ethercat_dirs:
            fpath = os.path.join(d, fname)
            if not os.path.isfile(fpath):
                results.append((fname, d, False, "File not found"))
                continue

            try:
                with open(fpath, "r") as f:
                    content = f.read()

                # Negate all factor values in channel lines
                new_content = re.sub(
                    r"factor: ([+-]?[\d.e-]+)",
                    _negate_factor,
                    content,
                )

                # Update Direction comment: flip sign
                def flip_direction_comment(m):
                    val = int(m.group(1))
                    return f"# Direction: {-val}"

                new_content = re.sub(
                    r"# Direction:\s*([+-]?\d+)",
                    flip_direction_comment,
                    new_content,
                )

                # Update description text
                new_content = new_content.replace(
                    "Positive = uncalibrated direction",
                    "Negative = reversed direction",
                )
                new_content = new_content.replace(
                    "Negative = reversed direction",
                    "Negative = reversed direction",
                )
                new_content = new_content.replace(
                    "Positive = reversed direction",
                    "Negative = reversed direction",
                )

                # Count negated factors for verification
                neg_count = len(re.findall(r"factor: -", new_content))
                pos_count = len(re.findall(r"factor: [^-]", new_content))
                # In a properly reversed file, all 6 factors should have the same sign

                dirname = os.path.basename(d)
                if dry_run:
                    results.append((
                        fname, dirname, True,
                        f"[dry-run] {neg_count} negative, {pos_count} positive factors"
                    ))
                else:
                    with open(fpath, "w") as f:
                        f.write(new_content)
                    results.append((
                        fname, dirname, True,
                        f"{neg_count} negative, {pos_count} positive factors"
                    ))

            except Exception as e:
                results.append((fname, os.path.basename(d), False, str(e)))

    return results


def list_directions(ethercat_dirs=None):
    """List the current direction of all joint config files.

    Returns list of (filename, direction_int, joint_name) tuples.
    """
    if ethercat_dirs is None:
        ethercat_dirs = find_ethercat_dirs()

    # Use the first (primary) directory
    if not ethercat_dirs:
        return []

    d = ethercat_dirs[0]
    entries = []

    for fname in sorted(os.listdir(d)):
        if not fname.endswith(".yaml"):
            continue

        fpath = os.path.join(d, fname)
        with open(fpath, "r") as f:
            content = f.read()

        # Parse direction from comment
        m = re.search(r"# Direction:\s*([+-]?\d+)", content)
        direction = int(m.group(1)) if m else 0

        # Parse joint name from comment
        m = re.search(r"#\s*ICube EtherCAT slave config\s*[—–-]\s*(\S+)", content)
        joint_name = m.group(1) if m else fname.replace(".yaml", "")

        entries.append((fname, direction, joint_name))

    return entries


def main():
    parser = argparse.ArgumentParser(
        description="Reverse motor direction in EtherCAT slave YAML configs"
    )
    parser.add_argument(
        "files", nargs="*",
        help="YAML filenames to reverse (e.g. right_shoulder_roll_X6.yaml)",
    )
    parser.add_argument(
        "--ethercat-dir", action="append",
        help="Path to ethercat config directory (auto-detected if omitted). "
             "Can be specified multiple times.",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Preview changes without writing files",
    )
    parser.add_argument(
        "--list", action="store_true", dest="list_all",
        help="List current direction of all joints and exit",
    )
    args = parser.parse_args()

    dirs = args.ethercat_dir if args.ethercat_dir else None

    if args.list_all:
        entries = list_directions(dirs)
        if not entries:
            print("No config files found.")
            sys.exit(1)
        print(f"{'File':<40} {'Dir':>4}  Joint")
        print("-" * 80)
        for fname, direction, joint_name in entries:
            sign = f"{direction:+d}"
            print(f"  {fname:<38} {sign:>4}  {joint_name}")
        sys.exit(0)

    if not args.files:
        parser.error("No files specified. Use --list to see available files.")

    results = reverse_direction(args.files, ethercat_dirs=dirs, dry_run=args.dry_run)

    ok = 0
    fail = 0
    for fname, dirname, success, msg in results:
        status = "OK" if success else "FAIL"
        print(f"  {status}: {dirname}/{fname} — {msg}")
        if success:
            ok += 1
        else:
            fail += 1

    print(f"\n{'[DRY RUN] ' if args.dry_run else ''}"
          f"Reversed {ok} files, {fail} failures.")
    if not args.dry_run and ok > 0:
        print("NOTE: Offsets must be recalibrated for reversed joints.")
    sys.exit(0 if fail == 0 else 1)


if __name__ == "__main__":
    main()
