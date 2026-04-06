#!/usr/bin/env python3
"""
Generate a locomotion-RL–ready URDF from full_robot.urdf.xacro for IsaacLab.

Key differences from generate_urdf.py (dual-arm):
  - Uses full_robot_description (legs + torso + arms + hands)
  - Removes world/world_joint so urdf_base is a free-floating root
  - Converts hand joints to fixed (reduces action space for locomotion RL)
  - Same GLB→OBJ conversion, mimic removal, dynamics & effort tweaks

Usage (ROS 2 workspace must be sourced):
    source ~/Git/ldr-harambe-arms-sim2real/install/setup.bash
    python3 generate_urdf_rl.py --output ~/harambe_rl/harambe.urdf
"""

import argparse
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

# Reuse helpers from the existing generate_urdf.py in the same directory
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)
from generate_urdf import (  # noqa: E402
    convert_glb_meshes_in_urdf,
    increase_hand_joint_dynamics,
    increase_hand_link_inertia,
    increase_joint_effort_limits,
    remove_hand_mimic_tags,
    remove_thumb_collision,
)

# ---------------------------------------------------------------------------
# Package URI resolution using ament_index_python (avoids ros2 subprocess)
# ---------------------------------------------------------------------------

def _get_pkg_share(pkg_name: str) -> str | None:
    """Resolve a package share directory via ament_index_python or AMENT_PREFIX_PATH."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(pkg_name)
    except Exception:
        pass
    # Fallback: scan AMENT_PREFIX_PATH manually
    prefix_path = os.environ.get("AMENT_PREFIX_PATH", "")
    for prefix in prefix_path.split(":"):
        share = os.path.join(prefix, "share", pkg_name)
        if os.path.isdir(share):
            return share
    return None


def resolve_package_uri(match: re.Match) -> str:
    """Replace package://pkg_name/path with the absolute filesystem path."""
    pkg_name = match.group(1)
    rel_path = match.group(2)
    share = _get_pkg_share(pkg_name)
    if share:
        return os.path.join(share, rel_path)
    print(f"[WARN] Could not resolve package://{pkg_name}/{rel_path}")
    return match.group(0)


# ---------------------------------------------------------------------------
# RL-specific post-processing
# ---------------------------------------------------------------------------

def remove_world_joint(urdf_content: str) -> str:
    """Remove the world link and world_joint so urdf_base is the free root.

    full_robot.urdf.xacro pins the pelvis to the world frame via a fixed joint.
    IsaacLab / PhysX needs the root link to have no parent so it can be a
    free-floating articulation root.  We strip the dummy link entirely.
    """
    root = ET.fromstring(urdf_content)

    # Remove the <link name="world"/> element
    for link in root.findall("link"):
        if link.get("name") == "world":
            root.remove(link)
            break

    # Remove the <joint name="world_joint"> element
    for joint in root.findall("joint"):
        if joint.get("name") == "world_joint":
            root.remove(joint)
            break

    return ET.tostring(root, encoding="unicode", xml_declaration=True)


def fix_hand_joints(urdf_content: str) -> str:
    """Convert all hand/finger joints to fixed type for locomotion RL.

    Including 24 hand DOF in a locomotion policy inflates the action space
    without contributing to the walking reward.  Fixing them keeps the
    visual geometry intact while reducing controllable DOF to 25
    (1 waist + 12 arms + 12 legs).
    """
    HAND_KEYWORDS = ("thumb", "index", "middle", "ring", "pinky")
    root = ET.fromstring(urdf_content)
    count = 0

    for joint in root.iter("joint"):
        name = joint.get("name", "").lower()
        if not any(kw in name for kw in HAND_KEYWORDS):
            continue
        if joint.get("type") in ("revolute", "prismatic", "continuous"):
            joint.set("type", "fixed")
            # Remove child elements that are only meaningful for moving joints
            for tag in ("limit", "dynamics", "axis", "mimic"):
                el = joint.find(tag)
                if el is not None:
                    joint.remove(el)
            count += 1

    if count:
        print(f"[INFO] Fixed {count} hand joints (zero DOF for locomotion RL)")

    return ET.tostring(root, encoding="unicode", xml_declaration=True)


# ---------------------------------------------------------------------------
# Main generation
# ---------------------------------------------------------------------------

def generate_urdf_rl(output_path: str) -> str:
    """Generate the full-robot RL URDF.

    Steps
    -----
    1. Run xacro on full_robot.urdf.xacro with all legs and waist active.
    2. Resolve package:// URIs to absolute paths.
    3. Convert GLB meshes → OBJ (bake node transforms for PhysX).
    4. Remove world link + world_joint → urdf_base becomes free root.
    5. Fix hand joints (zero DOF) to keep locomotion action space small.
    6. Remove thumb collisions (geometry overlaps palm at 0°).
    7. Remove any remaining mimic tags.
    8. Bump hand joint dynamics for PhysX stability.
    9. Raise effort limits on all joints (URDF effort → PhysX maxForce).
    10. Raise hand link inertia for acceleration-mode drives.
    """
    # Locate xacro entry point
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "--share", "full_robot_description"],
            capture_output=True, text=True, check=True,
        )
        xacro_path = os.path.join(result.stdout.strip(), "urdf", "full_robot.urdf.xacro")
    except (subprocess.CalledProcessError, FileNotFoundError):
        # Fallback: relative to this script (source tree)
        xacro_path = os.path.normpath(os.path.join(
            _SCRIPT_DIR, "..", "..", "..", "robot_description",
            "full_robot_description", "urdf", "full_robot.urdf.xacro",
        ))

    if not os.path.isfile(xacro_path):
        sys.exit(f"[ERROR] xacro file not found: {xacro_path}")

    print(f"[INFO] Running xacro: {xacro_path}")
    result = subprocess.run(
        [
            "xacro", xacro_path,
            "fixed_legs:=false",   # legs are active revolute joints
            "only_left:=false",    # waist_yaw is active revolute
        ],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        sys.exit(f"[ERROR] xacro failed:\n{result.stderr}")

    # Resolve package:// URIs
    urdf_content = re.sub(
        r"package://([^/]+)/([^\"<\s]+)",
        resolve_package_uri,
        result.stdout,
    )

    output_dir = os.path.dirname(os.path.abspath(output_path))

    # Step 3: GLB → OBJ (bake transforms so PhysX sees correct geometry)
    urdf_content = convert_glb_meshes_in_urdf(urdf_content, output_dir)

    # Step 4: Drop world link/joint → urdf_base becomes free-floating root
    urdf_content = remove_world_joint(urdf_content)

    # Step 5: Fix hand joints (locomotion policy doesn't need finger control)
    urdf_content = fix_hand_joints(urdf_content)

    # Step 6: Remove thumb collision (overlaps palm at 0°)
    urdf_content = remove_thumb_collision(urdf_content)

    # Step 7: Remove remaining mimic tags
    urdf_content = remove_hand_mimic_tags(urdf_content)

    # Step 8: Bump hand joint dynamics for PhysX
    urdf_content = increase_hand_joint_dynamics(urdf_content)

    # Step 9: Raise effort limits (URDF effort → PhysX maxForce)
    urdf_content = increase_joint_effort_limits(urdf_content)

    # Step 10: Minimum hand link inertia for acceleration-mode drives
    urdf_content = increase_hand_link_inertia(urdf_content)

    os.makedirs(output_dir, exist_ok=True)
    with open(output_path, "w") as f:
        f.write(urdf_content)

    print(f"[INFO] RL URDF written to: {output_path}")
    print("[INFO] Active DOF: 1 waist + 12 arms + 12 legs = 25 joints")
    print("[INFO] Hand joints: fixed (merged into wrist by IsaacLab converter)")
    return output_path


def main():
    parser = argparse.ArgumentParser(
        description="Generate full-robot RL URDF with absolute paths for IsaacLab"
    )
    default_out = os.path.expanduser("~/harambe_rl/harambe.urdf")
    parser.add_argument(
        "--output", "-o", type=str, default=default_out,
        help=f"Output URDF path (default: {default_out})",
    )
    args = parser.parse_args()
    generate_urdf_rl(args.output)


if __name__ == "__main__":
    main()
