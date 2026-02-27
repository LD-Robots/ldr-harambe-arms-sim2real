#!/usr/bin/env bash
set -euo pipefail

# Generate a static URDF from xacro for mc_rtc / mc_mujoco
# Requires: ROS 2 workspace built and sourced (source install/setup.bash)
#
# Outputs:
#   urdf/harambe_mc_rtc.urdf  — URDF with absolute mesh paths (for mc_rtc)
#   urdf/harambe_mujoco.xml   — MuJoCo XML with embedded meshes (for mc_mujoco)

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
WORKSPACE_ROOT="$(cd "$PKG_DIR/../../.." && pwd)"
OUTPUT_DIR="$PKG_DIR/urdf"

mkdir -p "$OUTPUT_DIR"

XACRO_FILE="$WORKSPACE_ROOT/src/robot_description/full_robot_description/urdf/full_robot.urdf.xacro"

if [ ! -f "$XACRO_FILE" ]; then
  echo "ERROR: Xacro file not found: $XACRO_FILE"
  echo "Make sure you are in the correct workspace."
  exit 1
fi

if ! command -v xacro &>/dev/null; then
  echo "ERROR: xacro not found. Source your ROS workspace first:"
  echo "  source install/setup.bash"
  exit 1
fi

TEMP_FILE="$OUTPUT_DIR/harambe_full.urdf"
FINAL_FILE="$OUTPUT_DIR/harambe_mc_rtc.urdf"
MUJOCO_XML="$OUTPUT_DIR/harambe_mujoco.xml"

# === Step 1: Generate URDF from xacro ===
echo "Generating URDF from xacro (all joints revolute)..."
xacro "$XACRO_FILE" \
  fixed_legs:=false \
  only_left:=false \
  > "$TEMP_FILE"

echo "Removing world link and world_joint for free-floating base..."
python3 - "$TEMP_FILE" "$FINAL_FILE" << 'PYEOF'
import sys
import xml.etree.ElementTree as ET

tree = ET.parse(sys.argv[1])
root = tree.getroot()

for joint in root.findall('joint'):
    if joint.get('name') == 'world_joint':
        root.remove(joint)

for link in root.findall('link'):
    if link.get('name') == 'world':
        root.remove(link)

tree.write(sys.argv[2], xml_declaration=True, encoding='unicode')
PYEOF

rm -f "$TEMP_FILE"

# === Step 2: Resolve package:// to absolute paths ===
ROBOT_DESC_DIR="$WORKSPACE_ROOT/src/robot_description"
echo "Resolving package:// URIs to absolute paths..."
sed -i \
  -e "s|package://full_robot_description|$ROBOT_DESC_DIR/full_robot_description|g" \
  -e "s|package://hand_description|$ROBOT_DESC_DIR/hand_description|g" \
  "$FINAL_FILE"

echo "Generated: $FINAL_FILE (mc_rtc URDF with absolute mesh paths)"

# === Step 3: Convert URDF to MuJoCo XML ===
# MuJoCo's URDF compiler resolves meshes relative to the URDF directory,
# so we create temporary symlinks for compilation then clean up.
echo ""
echo "Converting URDF to MuJoCo XML..."
python3 - "$FINAL_FILE" "$MUJOCO_XML" "$OUTPUT_DIR" << 'PYEOF'
import sys, os, re
import mujoco

urdf_path = sys.argv[1]
xml_path = sys.argv[2]
output_dir = sys.argv[3]

# Read URDF and find all mesh absolute paths
with open(urdf_path) as f:
    content = f.read()
mesh_abs = set(re.findall(r'filename="([^"]+\.(?:stl|STL))"', content))
print(f"  Found {len(mesh_abs)} unique mesh references")

# Create temporary symlinks for MuJoCo URDF compilation
temp_links = []
for abs_path in mesh_abs:
    basename = os.path.basename(abs_path)
    link_path = os.path.join(output_dir, basename)
    if not os.path.exists(link_path) and os.path.exists(abs_path):
        os.symlink(abs_path, link_path)
        temp_links.append(link_path)

# Compile URDF -> XML
model = mujoco.MjModel.from_xml_path(urdf_path)
mujoco.mj_saveLastXML(xml_path, model)
print(f"  MuJoCo model: {model.nq} qpos, {model.nv} dof, {model.nbody} bodies")

# Build basename -> absolute path map
basename_to_abs = {}
for abs_path in mesh_abs:
    name_no_ext = os.path.splitext(os.path.basename(abs_path))[0]
    basename_to_abs[name_no_ext] = abs_path

# Restore absolute paths in XML
import xml.etree.ElementTree as ET
tree = ET.parse(xml_path)
root = tree.getroot()
restore_count = 0
for mesh_el in root.iter('mesh'):
    mname = mesh_el.get('name')
    if mname in basename_to_abs:
        mesh_el.set('file', basename_to_abs[mname])
        restore_count += 1
ET.indent(tree, space='  ')
tree.write(xml_path, xml_declaration=True, encoding='unicode')
print(f"  Restored {restore_count} absolute mesh paths in XML")

# Clean up temporary symlinks
for link_path in temp_links:
    os.unlink(link_path)
print(f"  Cleaned up {len(temp_links)} temporary symlinks")
PYEOF
echo "Generated: $MUJOCO_XML (for mc_mujoco)"
rm -f "$OUTPUT_DIR/harambe_mujoco.urdf"

# === Step 4: Fix body hierarchy + inject sensors ===
# MuJoCo's URDF compiler merges the root link (urdf_base) into worldbody,
# splitting the robot into disconnected kinematic trees. We fix this by
# wrapping all worldbody contents in a proper urdf_base body.
# Also add force/torque/IMU sensors (without robot prefix — mc_mujoco adds it).
echo "Fixing root body hierarchy and injecting sensors..."
python3 - "$MUJOCO_XML" "$FINAL_FILE" << 'PYEOF'
import sys
import xml.etree.ElementTree as ET

mj_path = sys.argv[1]
urdf_path = sys.argv[2]

# --- Parse urdf_base inertial from the URDF ---
urdf_tree = ET.parse(urdf_path)
urdf_root = urdf_tree.getroot()
base_link = None
for link in urdf_root.findall('link'):
    if link.get('name') == 'urdf_base':
        base_link = link
        break

base_mass = '5.0'
base_com = '0 0 0'
base_diaginertia = '0.03 0.02 0.02'
if base_link is not None:
    inertial = base_link.find('inertial')
    if inertial is not None:
        origin = inertial.find('origin')
        mass_el = inertial.find('mass')
        inertia_el = inertial.find('inertia')
        if origin is not None:
            base_com = origin.get('xyz', '0 0 0')
        if mass_el is not None:
            base_mass = mass_el.get('value', '5.0')
        if inertia_el is not None:
            ixx = inertia_el.get('ixx', '0.03')
            iyy = inertia_el.get('iyy', '0.02')
            izz = inertia_el.get('izz', '0.02')
            base_diaginertia = f'{ixx} {iyy} {izz}'

print(f"  urdf_base: mass={base_mass}, com={base_com}")

# --- Fix MuJoCo XML body hierarchy ---
mj_tree = ET.parse(mj_path)
mj_root = mj_tree.getroot()
worldbody = mj_root.find('worldbody')

# Create root body for urdf_base, positioned at standing height
root_body = ET.Element('body')
root_body.set('name', 'urdf_base')
root_body.set('pos', '0 0 1.26')

# Add inertial
inertial = ET.SubElement(root_body, 'inertial')
inertial.set('pos', base_com)
inertial.set('mass', base_mass)
inertial.set('diaginertia', base_diaginertia)

# Add freejoint so the robot is free-floating (not fixed in space)
fj = ET.SubElement(root_body, 'freejoint')
fj.set('name', 'root')
print("  Added freejoint 'root' to urdf_base")

# Move all children of worldbody into root_body
children = list(worldbody)
for child in children:
    worldbody.remove(child)
    root_body.append(child)

# Insert root_body as sole child of worldbody
worldbody.append(root_body)
print("  Wrapped worldbody contents in <body name='urdf_base'>")

# --- Add sensor sites ---
body_sites = {
    'urdf_foot_assembly':    [('LeftFoot_site',  '0.04 0 -0.02')],
    'urdf_foot_assembly_2':  [('RightFoot_site', '0.04 0 -0.02')],
    'urdf_simplified_torso': [('Default_site',   '0 0 0')],
}
for body in worldbody.iter('body'):
    bname = body.get('name')
    if bname in body_sites:
        for site_name, site_pos in body_sites[bname]:
            site = ET.SubElement(body, 'site')
            site.set('name', site_name)
            site.set('pos', site_pos)
            site.set('size', '0.005')
            print(f"  Added site '{site_name}' to body '{bname}'")

# --- Strip joint damping and frictionloss (G1 has none; these fight PD control) ---
strip_count = 0
for body in worldbody.iter('body'):
    for jnt in body.findall('joint'):
        for attr in ('damping', 'frictionloss', 'actuatorfrcrange'):
            if attr in jnt.attrib:
                del jnt.attrib[attr]
                strip_count += 1
print(f"  Stripped {strip_count} damping/frictionloss/actuatorfrcrange attributes from joints")

# --- Add motor actuators for every hinge joint (matching G1: name=joint_name) ---
# mc_mujoco needs <motor> elements to control joints via PD gains
actuator_sec = mj_root.find('actuator')
if actuator_sec is None:
    actuator_sec = ET.SubElement(mj_root, 'actuator')
motor_count = 0
for body in worldbody.iter('body'):
    for jnt in body.findall('joint'):
        jnt_name = jnt.get('name')
        jnt_type = jnt.get('type', 'hinge')  # MuJoCo default is hinge
        if jnt_type == 'hinge' and jnt_name:
            motor = ET.SubElement(actuator_sec, 'motor')
            motor.set('name', jnt_name)
            motor.set('joint', jnt_name)
            motor_count += 1
print(f"  Added {motor_count} motor actuators")

# --- Add sensors (mc_mujoco prefixes with "Harambe_", so use bare names) ---
sensor_sec = ET.SubElement(mj_root, 'sensor')
sensors = [
    ('force',         'LeftFoot_fsensor',      'LeftFoot_site'),
    ('torque',        'LeftFoot_tsensor',      'LeftFoot_site'),
    ('force',         'RightFoot_fsensor',     'RightFoot_site'),
    ('torque',        'RightFoot_tsensor',     'RightFoot_site'),
    ('gyro',          'Default_gyro',          'Default_site'),
    ('accelerometer', 'Default_accelerometer', 'Default_site'),
]
for stype, sname, ssite in sensors:
    el = ET.SubElement(sensor_sec, stype)
    el.set('name', sname)
    el.set('site', ssite)
    print(f"  Added {stype} sensor '{sname}'")

ET.indent(mj_tree, space='  ')
mj_tree.write(mj_path, xml_declaration=True, encoding='unicode')
PYEOF
echo "Fixed body hierarchy and injected sensors into: $MUJOCO_XML"

# === Summary ===
echo ""
echo "Joint count:"
python3 -c "
import xml.etree.ElementTree as ET
tree = ET.parse('$FINAL_FILE')
root = tree.getroot()
revolute = [j.get('name') for j in root.findall('joint') if j.get('type') == 'revolute']
mimic = [j.get('name') for j in root.findall('joint') if j.find('mimic') is not None]
fixed = [j.get('name') for j in root.findall('joint') if j.get('type') == 'fixed']
print(f'  Revolute: {len(revolute)} ({len(revolute) - len(mimic)} actuated + {len(mimic)} mimic)')
print(f'  Fixed: {len(fixed)}')
print(f'  Total: {len(revolute) + len(fixed)}')
"
