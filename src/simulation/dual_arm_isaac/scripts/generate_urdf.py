#!/usr/bin/env python3
"""
Generate the dual-arm URDF with absolute mesh paths for Isaac Sim.

This script must run with the SYSTEM Python (not Isaac Sim's Python)
so that xacro and ros2 CLI tools work correctly.

Usage:
    source ~/colcon_ws/install/setup.bash
    python3 generate_urdf.py --output /tmp/dual_arm_isaac.urdf
"""

import argparse
import os
import re
import subprocess
import sys


def resolve_package_uri(match):
    """Replace package://pkg_name/path with the absolute filesystem path."""
    pkg_name = match.group(1)
    rel_path = match.group(2)
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "--share", pkg_name],
            capture_output=True, text=True, check=True,
        )
        return os.path.join(result.stdout.strip(), rel_path)
    except (subprocess.CalledProcessError, FileNotFoundError):
        print(f"[WARN] Could not resolve package://{pkg_name}/{rel_path}")
        return match.group(0)


def generate_urdf(output_path):
    """Generate the dual-arm URDF with absolute mesh paths."""
    # Locate the xacro entry point
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "--share", "dual_arm_description"],
            capture_output=True, text=True, check=True,
        )
        xacro_path = os.path.join(
            result.stdout.strip(), "urdf", "dual_arm.urdf.xacro"
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        # Fallback: try relative to this script (source tree)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        xacro_path = os.path.join(
            script_dir, "..", "..", "..", "robot_description",
            "dual_arm_description", "urdf", "dual_arm.urdf.xacro",
        )
        xacro_path = os.path.normpath(xacro_path)

    if not os.path.isfile(xacro_path):
        sys.exit(f"[ERROR] xacro file not found: {xacro_path}")

    print(f"[INFO] Running xacro: {xacro_path}")
    result = subprocess.run(
        ["xacro", xacro_path, "use_isaac:=true"],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        sys.exit(f"[ERROR] xacro failed:\n{result.stderr}")

    # Replace every package://pkg/path with an absolute path
    urdf_content = re.sub(
        r"package://([^/]+)/([^\"<\s]+)",
        resolve_package_uri,
        result.stdout,
    )

    # Write output
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, "w") as f:
        f.write(urdf_content)

    print(f"[INFO] URDF written to: {output_path}")
    return output_path


def main():
    parser = argparse.ArgumentParser(
        description="Generate dual-arm URDF with absolute paths for Isaac Sim"
    )
    parser.add_argument(
        "--output", "-o", type=str, default="/tmp/dual_arm_isaac.urdf",
        help="Output URDF file path (default: /tmp/dual_arm_isaac.urdf)",
    )
    args = parser.parse_args()
    generate_urdf(args.output)


if __name__ == "__main__":
    main()
