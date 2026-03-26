"""Embedded PyVista 3D robot viewer — loads URDF, renders meshes, updates
joint angles in real-time from /joint_states."""

import math
import os

import numpy as np

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QLabel, QVBoxLayout, QWidget

# Catppuccin Mocha
_BG = "#1e1e2e"
_BG_RGB = (0.118, 0.118, 0.180)
_SUBTEXT = "#6c7086"
_HIGHLIGHT = (1.0, 0.6, 0.2)  # Orange for highlighted joint
_DEFAULT_COLOR = (0.7, 0.7, 0.75)  # Default mesh color


def _rotation_matrix(axis, angle):
    """Rotation matrix around an arbitrary axis (Rodrigues formula)."""
    ax = np.array(axis, dtype=float)
    ax = ax / (np.linalg.norm(ax) + 1e-12)
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    x, y, z = ax
    return np.array([
        [t * x * x + c,     t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c,     t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ])


def _rpy_to_matrix(r, p, y):
    """Roll-pitch-yaw (XYZ intrinsic) to 3x3 rotation matrix."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _make_transform(xyz, rpy):
    """Create 4x4 homogeneous transform from xyz + rpy."""
    T = np.eye(4)
    T[:3, :3] = _rpy_to_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def _resolve_mesh_path(uri, ros_node=None):
    """Resolve ``package://pkg_name/path`` URI to filesystem path."""
    if not uri.startswith("package://"):
        return uri

    rest = uri[len("package://"):]
    pkg_name = rest.split("/")[0]
    rel_path = "/".join(rest.split("/")[1:])

    # Try ament_index first (installed workspace)
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_dir = get_package_share_directory(pkg_name)
        resolved = os.path.join(pkg_dir, rel_path)
        if os.path.exists(resolved):
            return resolved
    except Exception:
        pass

    # Fallback: search common source locations relative to this file
    tool_dir = os.path.dirname(os.path.abspath(__file__))
    # Walk up to workspace root: ethercat_tools/ethercat_tools/joint_calibration/ -> src/tools/ethercat_tools/
    ws_root = os.path.abspath(os.path.join(tool_dir, "..", "..", "..", "..", ".."))
    # Search for package in src/
    for dirpath, dirnames, _ in os.walk(os.path.join(ws_root, "src")):
        if os.path.basename(dirpath) == pkg_name:
            candidate = os.path.join(dirpath, rel_path)
            if os.path.exists(candidate):
                return candidate
        # Limit depth
        depth = dirpath[len(os.path.join(ws_root, "src")):].count(os.sep)
        if depth > 3:
            dirnames.clear()

    return uri  # Return as-is if can't resolve


class RobotViewer(QWidget):
    """Embedded 3D robot viewer using PyVista."""

    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self._ros_node = ros_node
        self._joint_angles = {}
        self._highlighted_joint = None

        # URDF data
        self._urdf = None
        self._joint_map = {}      # {joint_name: urdf.Joint}
        self._link_map = {}       # {link_name: urdf.Link}
        self._parent_map = {}     # {child_link: (joint, parent_link)}
        self._mesh_actors = {}    # {link_name: [(actor, visual_origin_T)]}
        self._link_colors = {}    # {link_name: original_color}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        try:
            import pyvista as pv
            from pyvistaqt import QtInteractor

            pv.global_theme.background = _BG_RGB
            pv.global_theme.font.color = "white"

            self._plotter = QtInteractor(self)
            layout.addWidget(self._plotter.interactor)

            self._load_urdf()

        except ImportError as e:
            self._plotter = None
            lbl = QLabel(
                f"3D viewer requires pyvista:\n{e}\n\n"
                "pip install pyvista pyvistaqt vtk"
            )
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(f"color: {_SUBTEXT}; font-size: 12px;")
            lbl.setWordWrap(True)
            layout.addWidget(lbl)

    def _load_urdf(self):
        """Load URDF from /robot_description parameter and build meshes."""
        if self._plotter is None:
            return

        urdf_string = self._get_urdf_string()
        if not urdf_string:
            self._show_message("Waiting for /robot_description parameter...")
            return

        try:
            from urdf_parser_py.urdf import URDF
            self._urdf = URDF.from_xml_string(urdf_string)
        except Exception as e:
            self._show_message(f"Failed to parse URDF:\n{e}")
            return

        # Build lookup maps
        for joint in self._urdf.joints:
            self._joint_map[joint.name] = joint
            self._parent_map[joint.child] = (joint, joint.parent)

        for link in self._urdf.links:
            self._link_map[link.name] = link

        # Load and display meshes
        self._load_meshes()

        # Set camera
        self._plotter.reset_camera()
        self._plotter.camera.position = (0.8, 0.8, 0.8)
        self._plotter.camera.focal_point = (0, 0.2, 0.2)
        self._plotter.camera.up = (0, 0, 1)

    def _get_urdf_string(self):
        """Get URDF string from /robot_description ROS parameter."""
        if self._ros_node is None:
            return None

        try:
            # Try to get robot_description from parameter server via topic
            from rcl_interfaces.srv import GetParameters
            client = self._ros_node.create_client(
                GetParameters, "/robot_state_publisher/get_parameters"
            )
            if not client.wait_for_service(timeout_sec=2.0):
                return None

            from rcl_interfaces.msg import ParameterType
            request = GetParameters.Request()
            request.names = ["robot_description"]
            future = client.call_async(request)

            import rclpy
            rclpy.spin_until_future_complete(self._ros_node, future, timeout_sec=3.0)

            if future.result() is not None:
                values = future.result().values
                if values and values[0].type == ParameterType.PARAMETER_STRING:
                    return values[0].string_value
        except Exception:
            pass

        # Fallback: try reading from topic
        try:
            from std_msgs.msg import String
            import rclpy

            urdf_holder = [None]

            def _cb(msg):
                urdf_holder[0] = msg.data

            sub = self._ros_node.create_subscription(
                String, "/robot_description", _cb, 1
            )
            # Spin briefly to receive
            for _ in range(30):
                rclpy.spin_once(self._ros_node, timeout_sec=0.1)
                if urdf_holder[0]:
                    break
            self._ros_node.destroy_subscription(sub)
            return urdf_holder[0]
        except Exception:
            pass

        return None

    def _load_meshes(self):
        """Load STL/OBJ meshes for all links and add to plotter."""
        import pyvista as pv

        for link in self._urdf.links:
            if not link.visuals:
                continue

            actors = []
            for visual in link.visuals:
                if visual.geometry is None:
                    continue

                mesh = None
                color = _DEFAULT_COLOR

                # Get mesh geometry
                if hasattr(visual.geometry, "filename"):
                    mesh_path = _resolve_mesh_path(
                        visual.geometry.filename, self._ros_node
                    )
                    if os.path.exists(mesh_path):
                        try:
                            mesh = pv.read(mesh_path)
                        except Exception:
                            continue
                    else:
                        continue
                elif hasattr(visual.geometry, "size"):
                    # Box
                    s = visual.geometry.size
                    mesh = pv.Box(bounds=[
                        -s[0]/2, s[0]/2, -s[1]/2, s[1]/2, -s[2]/2, s[2]/2
                    ])
                elif hasattr(visual.geometry, "radius") and hasattr(visual.geometry, "length"):
                    # Cylinder
                    mesh = pv.Cylinder(
                        radius=visual.geometry.radius,
                        height=visual.geometry.length,
                    )
                elif hasattr(visual.geometry, "radius") and not hasattr(visual.geometry, "length"):
                    # Sphere
                    mesh = pv.Sphere(radius=visual.geometry.radius)
                else:
                    continue

                if mesh is None:
                    continue

                # Get visual color
                if visual.material and visual.material.color:
                    rgba = visual.material.color.rgba
                    if rgba and len(rgba) >= 3:
                        color = (rgba[0], rgba[1], rgba[2])

                # Visual origin transform (within the link frame)
                vis_origin_T = np.eye(4)
                if visual.origin:
                    xyz = visual.origin.xyz if visual.origin.xyz else [0, 0, 0]
                    rpy = visual.origin.rpy if visual.origin.rpy else [0, 0, 0]
                    vis_origin_T = _make_transform(xyz, rpy)

                # Apply visual origin to mesh
                mesh.transform(vis_origin_T, inplace=True)

                actor = self._plotter.add_mesh(
                    mesh, color=color, smooth_shading=True,
                    opacity=0.9, name=f"{link.name}_{id(visual)}"
                )
                actors.append((actor, mesh, color))

            if actors:
                self._mesh_actors[link.name] = actors
                self._link_colors[link.name] = [a[2] for a in actors]

        # Apply initial joint angles (all zeros)
        self._apply_fk()

    def _get_link_transform(self, link_name):
        """Compute the world transform for a link by walking up the tree."""
        if link_name not in self._parent_map:
            return np.eye(4)

        # Build chain from root to this link
        chain = []
        current = link_name
        while current in self._parent_map:
            joint, parent = self._parent_map[current]
            chain.append(joint)
            current = parent

        chain.reverse()

        T = np.eye(4)
        for joint in chain:
            # Joint origin transform
            xyz = joint.origin.xyz if joint.origin and joint.origin.xyz else [0, 0, 0]
            rpy = joint.origin.rpy if joint.origin and joint.origin.rpy else [0, 0, 0]
            T_joint = _make_transform(xyz, rpy)
            T = T @ T_joint

            # Joint rotation (only for revolute/continuous)
            if joint.type in ("revolute", "continuous"):
                angle = self._joint_angles.get(joint.name, 0.0)
                axis = joint.axis if joint.axis else [1, 0, 0]
                R = np.eye(4)
                R[:3, :3] = _rotation_matrix(axis, angle)
                T = T @ R

        return T

    def _apply_fk(self):
        """Apply forward kinematics to update all mesh positions."""
        if self._plotter is None:
            return

        import pyvista as pv

        for link_name, actors in self._mesh_actors.items():
            T = self._get_link_transform(link_name)

            for actor, mesh, color in actors:
                # We need to reposition the actor. PyVista actors have a
                # user_matrix property.
                try:
                    actor.user_matrix = T
                except AttributeError:
                    # Fallback for older PyVista versions
                    try:
                        actor.SetUserMatrix(
                            pv.vtkmatrix_from_vtk_matrix(T)
                        )
                    except Exception:
                        pass

    def _show_message(self, text):
        """Show a text message in the 3D viewport."""
        if self._plotter is not None:
            self._plotter.add_text(
                text, position="upper_left", font_size=10, color="white"
            )

    def update_joints(self, joint_data):
        """Update joint angles from joint state data dict.

        Args:
            joint_data: dict of {joint_name: {position, velocity, effort}}
        """
        if self._plotter is None or self._urdf is None:
            return

        changed = False
        for name, data in joint_data.items():
            angle = data.get("position", 0.0)
            if name in self._joint_map:
                if self._joint_angles.get(name) != angle:
                    self._joint_angles[name] = angle
                    changed = True

        if changed:
            self._apply_fk()
            try:
                self._plotter.render()
            except Exception:
                pass

    def highlight_joint(self, joint_name):
        """Highlight the link associated with a joint."""
        if self._plotter is None or self._urdf is None:
            return

        if joint_name == self._highlighted_joint:
            return
        self._highlighted_joint = joint_name

        # Find child link of this joint
        highlight_link = None
        if joint_name in self._joint_map:
            highlight_link = self._joint_map[joint_name].child

        for link_name, actors in self._mesh_actors.items():
            original_colors = self._link_colors.get(link_name, [])
            for i, (actor, mesh, _) in enumerate(actors):
                if link_name == highlight_link:
                    try:
                        actor.prop.color = _HIGHLIGHT
                    except Exception:
                        pass
                else:
                    color = original_colors[i] if i < len(original_colors) else _DEFAULT_COLOR
                    try:
                        actor.prop.color = color
                    except Exception:
                        pass

        try:
            self._plotter.render()
        except Exception:
            pass

    def close(self):
        """Clean up plotter resources."""
        if self._plotter is not None:
            try:
                self._plotter.close()
            except Exception:
                pass
        super().close()
