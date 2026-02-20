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
import json
import os
import re
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np


# ---------------------------------------------------------------------------
# GLB → OBJ conversion  (bakes glTF node transforms into vertex positions)
# ---------------------------------------------------------------------------

def _quat_to_matrix(q):
    """Convert quaternion [x, y, z, w] to a 3×3 rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
        [  2*(x*y + z*w),   1 - 2*(x*x + z*z),   2*(y*z - x*w)],
        [  2*(x*z - y*w),     2*(y*z + x*w),   1 - 2*(x*x + y*y)],
    ])


def _read_accessor(accessor, buffer_views, bin_data):
    """Read data from a glTF accessor."""
    bv = buffer_views[accessor["bufferView"]]
    offset = bv.get("byteOffset", 0) + accessor.get("byteOffset", 0)
    count = accessor["count"]
    comp = accessor["componentType"]

    # Component type → numpy dtype
    dtype_map = {5120: np.int8, 5121: np.uint8, 5122: np.int16,
                 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
    dtype = dtype_map[comp]

    # Elements per item
    type_sizes = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}
    n = type_sizes.get(accessor["type"], 1)

    stride = bv.get("byteStride", 0)
    if stride and stride != dtype().itemsize * n:
        # Interleaved — read element by element
        item_size = np.dtype(dtype).itemsize * n
        arr = np.empty(count * n, dtype=dtype)
        for i in range(count):
            start = offset + i * stride
            arr[i * n:(i + 1) * n] = np.frombuffer(
                bin_data, dtype=dtype, count=n, offset=start)
        return arr.reshape(count, n) if n > 1 else arr
    else:
        arr = np.frombuffer(bin_data, dtype=dtype, count=count * n,
                            offset=offset)
        return arr.reshape(count, n) if n > 1 else arr


def convert_glb_to_obj(glb_path, obj_path):
    """Convert a GLB mesh to OBJ, baking node transforms into vertices.

    Isaac Sim's URDF importer may ignore glTF scene-graph node transforms
    (rotation, translation) embedded in GLB files.  By baking those
    transforms into the vertex positions and writing a simple OBJ file,
    the geometry always appears in the correct location regardless of
    how the importer handles the scene graph.
    """
    with open(glb_path, "rb") as f:
        magic, version, total_len = struct.unpack("<III", f.read(12))
        if magic != 0x46546C67:
            raise ValueError(f"Not a GLB file: {glb_path}")

        # JSON chunk
        json_len, json_type = struct.unpack("<II", f.read(8))
        gltf = json.loads(f.read(json_len).rstrip(b"\0"))

        # BIN chunk (pad to 4-byte boundary after JSON)
        f.seek(12 + 8 + json_len)
        pos = f.tell()
        if pos % 4:
            f.seek(pos + (4 - pos % 4))
        bin_len, bin_type = struct.unpack("<II", f.read(8))
        bin_data = f.read(bin_len)

    accessors = gltf["accessors"]
    buffer_views = gltf["bufferViews"]
    meshes = gltf["meshes"]
    nodes = gltf.get("nodes", [])

    all_verts = []
    all_normals = []
    all_faces = []
    vert_offset = 0

    for node in nodes:
        if "mesh" not in node:
            continue

        # Node transform
        rot = np.array(node.get("rotation", [0, 0, 0, 1]), dtype=np.float64)
        trans = np.array(node.get("translation", [0, 0, 0]), dtype=np.float64)
        scale = np.array(node.get("scale", [1, 1, 1]), dtype=np.float64)
        R = _quat_to_matrix(rot)

        mesh = meshes[node["mesh"]]
        for prim in mesh["primitives"]:
            attrs = prim["attributes"]

            # Vertex positions
            verts = _read_accessor(
                accessors[attrs["POSITION"]], buffer_views, bin_data
            ).astype(np.float64).copy()

            # Apply scale → rotation → translation
            verts *= scale
            verts = verts @ R.T
            verts += trans

            # Normals (rotate only, no translate/scale)
            normals = None
            if "NORMAL" in attrs:
                normals = _read_accessor(
                    accessors[attrs["NORMAL"]], buffer_views, bin_data
                ).astype(np.float64).copy()
                normals = normals @ R.T

            # Face indices
            if "indices" in prim:
                indices = _read_accessor(
                    accessors[prim["indices"]], buffer_views, bin_data
                ).astype(np.int64)
                faces = indices.reshape(-1, 3) + vert_offset
            else:
                n = len(verts)
                faces = np.arange(n).reshape(-1, 3) + vert_offset

            all_verts.append(verts)
            if normals is not None:
                all_normals.append(normals)
            all_faces.append(faces)
            vert_offset += len(verts)

    all_verts = np.concatenate(all_verts)
    all_faces = np.concatenate(all_faces)
    has_normals = len(all_normals) > 0
    if has_normals:
        all_normals = np.concatenate(all_normals)

    os.makedirs(os.path.dirname(obj_path) or ".", exist_ok=True)
    with open(obj_path, "w") as f:
        f.write(f"# Converted from {os.path.basename(glb_path)}\n")
        for v in all_verts:
            f.write(f"v {v[0]:.8f} {v[1]:.8f} {v[2]:.8f}\n")
        if has_normals:
            for n in all_normals:
                f.write(f"vn {n[0]:.8f} {n[1]:.8f} {n[2]:.8f}\n")
        for face in all_faces:
            a, b, c = face[0] + 1, face[1] + 1, face[2] + 1  # OBJ is 1-indexed
            if has_normals:
                f.write(f"f {a}//{a} {b}//{b} {c}//{c}\n")
            else:
                f.write(f"f {a} {b} {c}\n")


def convert_glb_meshes_in_urdf(urdf_content, output_dir):
    """Find all .glb mesh references in the URDF, convert them to .obj,
    and update the URDF to reference the .obj files instead.

    Converted OBJ files are stored in output_dir/meshes/.
    Returns the modified URDF content string.
    """
    root = ET.fromstring(urdf_content)
    mesh_dir = os.path.join(output_dir, "meshes")
    converted = {}  # glb_path → obj_path (cache to avoid re-converting)
    count = 0

    for mesh_el in root.iter("mesh"):
        filename = mesh_el.get("filename", "")
        if not filename.lower().endswith(".glb"):
            continue

        glb_path = filename  # already absolute after resolve_package_uri
        if not os.path.isfile(glb_path):
            print(f"[WARN] GLB file not found, skipping: {glb_path}")
            continue

        if glb_path not in converted:
            obj_name = os.path.splitext(os.path.basename(glb_path))[0] + ".obj"
            obj_path = os.path.join(mesh_dir, obj_name)
            convert_glb_to_obj(glb_path, obj_path)
            converted[glb_path] = obj_path
            count += 1

        mesh_el.set("filename", converted[glb_path])

    if count:
        print(f"[INFO] Converted {count} GLB meshes → OBJ in {mesh_dir}")

    return ET.tostring(root, encoding="unicode", xml_declaration=True)


def remove_thumb_collision(urdf_content):
    """Remove <collision> elements from thumb links in the URDF.

    The thumb geometry physically overlaps with the palm at joint position 0°
    (both with primitive boxes and with accurate OBJ meshes).  Isaac Sim's
    physics solver pushes the thumb to ~54° to resolve the penetration.
    Removing thumb collision entirely lets the joints stay at 0°.
    Visual meshes are kept — only collision shapes are stripped.
    """
    root = ET.fromstring(urdf_content)
    count = 0

    for link in root.iter("link"):
        name = link.get("name", "")
        if "thumb" not in name.lower():
            continue
        collisions = link.findall("collision")
        for col in collisions:
            link.remove(col)
            count += 1

    if count:
        print(f"[INFO] Removed {count} <collision> elements from thumb links")

    return ET.tostring(root, encoding="unicode", xml_declaration=True)


def increase_hand_joint_dynamics(urdf_content):
    """Increase damping and friction on hand/finger joints for Isaac Sim.

    The default values (damping=0.3, friction=0.05) are too low for Isaac Sim's
    PhysX solver — joints drift away from their drive targets.  Higher values
    help the joints hold their commanded positions.
    """
    HAND_KEYWORDS = ("thumb", "index", "middle", "ring", "pinky")
    root = ET.fromstring(urdf_content)
    count = 0

    for joint in root.iter("joint"):
        name = joint.get("name", "").lower()
        if not any(kw in name for kw in HAND_KEYWORDS):
            continue
        dyn = joint.find("dynamics")
        if dyn is None:
            dyn = ET.SubElement(joint, "dynamics")
        dyn.set("damping", "5.0")
        dyn.set("friction", "1.0")
        count += 1

    if count:
        print(f"[INFO] Increased dynamics on {count} hand joints (damping=5.0, friction=1.0)")

    return ET.tostring(root, encoding="unicode", xml_declaration=True)


def remove_hand_mimic_tags(urdf_content):
    """Remove <mimic> tags from hand/finger joints for Isaac Sim.

    The URDF importer creates mimic joints as PhysX constraints rather than
    driven joints.  These constraints are not stiff enough to resist gravity,
    so the intermediate/distal finger joints fall.  By removing the <mimic>
    tag, every finger joint becomes an independently driven revolute joint
    with its own PhysX drive (stiffness, damping, maxForce).
    """
    HAND_KEYWORDS = ("thumb", "index", "middle", "ring", "pinky")
    root = ET.fromstring(urdf_content)
    count = 0

    for joint in root.iter("joint"):
        name = joint.get("name", "").lower()
        if not any(kw in name for kw in HAND_KEYWORDS):
            continue
        mimic = joint.find("mimic")
        if mimic is not None:
            joint.remove(mimic)
            count += 1

    if count:
        print(f"[INFO] Removed <mimic> tags from {count} hand joints")

    return ET.tostring(root, encoding="unicode", xml_declaration=True)


def increase_hand_effort_limits(urdf_content):
    """Increase effort limits on hand/finger joints for Isaac Sim.

    The default effort limit (1.0 Nm) is mapped by Isaac Sim's URDF importer
    to the PhysX drive ``maxForce`` attribute.  Even with very high stiffness,
    the actual torque is clamped to ``maxForce``, so the drives cannot
    counteract gravity.  Setting a much higher value removes this bottleneck.
    """
    HAND_KEYWORDS = ("thumb", "index", "middle", "ring", "pinky")
    root = ET.fromstring(urdf_content)
    count = 0

    for joint in root.iter("joint"):
        name = joint.get("name", "").lower()
        if not any(kw in name for kw in HAND_KEYWORDS):
            continue
        limit = joint.find("limit")
        if limit is not None:
            limit.set("effort", "100.0")
            count += 1

    if count:
        print(f"[INFO] Increased effort limit to 100 Nm on {count} hand joints")

    return ET.tostring(root, encoding="unicode", xml_declaration=True)


def increase_hand_link_inertia(urdf_content):
    """Set a minimum inertia on hand/finger links for Isaac Sim.

    Isaac Sim's URDF importer creates **acceleration-mode** joint drives where
    ``torque = inertia × stiffness × position_error``.  Finger links have
    near-zero inertia (~1e-7 kg·m²), so even stiffness=1e5 produces
    negligible torque and the joints fall under gravity.

    Changing the drive type to "force" via USD attributes after physics
    initialisation does not propagate to the running PhysX solver.  The
    reliable fix is to give finger links a minimum diagonal inertia so the
    acceleration drives produce sufficient torque.

    A value of 1e-4 kg·m² with the default drive stiffness 1e4 gives
    ``torque = 1e-4 × 1e4 × error = 1.0 × error`` (N·m/rad), which is far
    more than the ~0.001 N·m gravitational torque on a finger link.
    """
    HAND_KEYWORDS = ("thumb", "index", "middle", "ring", "pinky")
    MIN_INERTIA = 1e-4   # kg·m²

    root = ET.fromstring(urdf_content)
    count = 0

    for link in root.iter("link"):
        name = link.get("name", "").lower()
        if not any(kw in name for kw in HAND_KEYWORDS):
            continue

        inertial = link.find("inertial")
        if inertial is None:
            inertial = ET.SubElement(link, "inertial")

        inertia = inertial.find("inertia")
        if inertia is None:
            inertia = ET.SubElement(inertial, "inertia")

        # Bump diagonal elements to at least MIN_INERTIA
        changed = False
        for attr in ("ixx", "iyy", "izz"):
            val = float(inertia.get(attr, "0"))
            if val < MIN_INERTIA:
                inertia.set(attr, str(MIN_INERTIA))
                changed = True

        # Ensure off-diagonal elements exist
        for attr in ("ixy", "ixz", "iyz"):
            if inertia.get(attr) is None:
                inertia.set(attr, "0")

        if changed:
            count += 1

    if count:
        print(f"[INFO] Set minimum inertia ({MIN_INERTIA} kg·m²) on {count} hand links")

    return ET.tostring(root, encoding="unicode", xml_declaration=True)


# ---------------------------------------------------------------------------
# Package URI resolution
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Main URDF generation
# ---------------------------------------------------------------------------

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

    # Convert GLB meshes to OBJ (bakes node transforms for Isaac Sim)
    output_dir = os.path.dirname(os.path.abspath(output_path))
    urdf_content = convert_glb_meshes_in_urdf(urdf_content, output_dir)

    # Remove thumb collision (geometry overlaps palm at 0°, pushing it to ~54°)
    urdf_content = remove_thumb_collision(urdf_content)

    # Remove mimic tags so ALL finger joints get independent drives
    # (mimic constraints are too weak to resist gravity in PhysX)
    urdf_content = remove_hand_mimic_tags(urdf_content)

    # Increase damping/friction on hand joints for Isaac Sim
    urdf_content = increase_hand_joint_dynamics(urdf_content)

    # Increase effort limits on hand joints (default 1 Nm is too low — Isaac Sim
    # maps effort to PhysX drive maxForce, which clamps drive torque)
    urdf_content = increase_hand_effort_limits(urdf_content)

    # Increase inertia on hand links so acceleration-mode drives produce enough
    # torque to counteract gravity (PhysX: torque = inertia × stiffness × error)
    urdf_content = increase_hand_link_inertia(urdf_content)

    # Write output
    os.makedirs(output_dir, exist_ok=True)
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
