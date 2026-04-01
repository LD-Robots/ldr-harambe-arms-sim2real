"""Build KDL kinematic chains from URDF for dynamics computation.

Provides utilities to parse a URDF string and construct a PyKDL Chain
between any two links, suitable for mass matrix and gravity torque
calculations via PyKDL.ChainDynParam.

Dependencies: PyKDL (python3-pykdl), urdf_parser_py
"""

from __future__ import annotations

import math
import sys

try:
    import PyKDL
except ImportError:
    sys.path.insert(0, "/usr/lib/python3/dist-packages")
    import PyKDL

from urdf_parser_py.urdf import URDF


def _pose_to_frame(pose) -> PyKDL.Frame:
    """Convert a URDF Pose (xyz + rpy) to a KDL Frame."""
    if pose is None:
        return PyKDL.Frame.Identity()
    x, y, z = pose.xyz if pose.xyz else (0, 0, 0)
    roll, pitch, yaw = pose.rpy if pose.rpy else (0, 0, 0)
    return PyKDL.Frame(
        PyKDL.Rotation.RPY(roll, pitch, yaw),
        PyKDL.Vector(x, y, z),
    )


def _inertial_to_kdl(inertial) -> PyKDL.RigidBodyInertia:
    """Convert a URDF Inertial element to KDL RigidBodyInertia.

    The URDF inertia tensor is specified at the center of mass in the
    link frame. KDL's RigidBodyInertia takes mass, center-of-gravity
    vector, and a RotationalInertia (about the CoG).
    """
    if inertial is None:
        return PyKDL.RigidBodyInertia()

    mass = inertial.mass or 0.0
    if mass == 0.0:
        return PyKDL.RigidBodyInertia()

    # CoG position in link frame
    origin = _pose_to_frame(inertial.origin)
    cog = origin.p

    i = inertial.inertia
    ixx = i.ixx or 0.0
    ixy = i.ixy or 0.0
    ixz = i.ixz or 0.0
    iyy = i.iyy or 0.0
    iyz = i.iyz or 0.0
    izz = i.izz or 0.0

    # KDL RotationalInertia constructor: (Ixx, Iyy, Izz, Ixy, Ixz, Iyz)
    rot_inertia = PyKDL.RotationalInertia(ixx, iyy, izz, ixy, ixz, iyz)

    return PyKDL.RigidBodyInertia(mass, cog, rot_inertia)


def _joint_to_kdl(joint) -> PyKDL.Joint:
    """Convert a URDF Joint to a KDL Joint.

    For revolute/continuous joints, the KDL joint axis is expressed in
    the joint's own frame (parent-to-child transform applied separately
    via the Segment's f_tip). The origin rotation is applied to the axis
    to transform it from the URDF joint frame to the parent link frame.
    """
    if joint.type == "fixed":
        return PyKDL.Joint(joint.name, PyKDL.Joint.Fixed)

    axis = PyKDL.Vector(*(joint.axis or (1, 0, 0)))
    origin = _pose_to_frame(joint.origin)

    # Rotate axis into parent frame
    rotated_axis = origin.M * axis

    if joint.type in ("revolute", "continuous"):
        return PyKDL.Joint(
            joint.name, origin.p, rotated_axis, PyKDL.Joint.RotAxis,
        )
    elif joint.type == "prismatic":
        return PyKDL.Joint(
            joint.name, origin.p, rotated_axis, PyKDL.Joint.TransAxis,
        )
    else:
        return PyKDL.Joint(joint.name, PyKDL.Joint.Fixed)


def build_chain(urdf_string: str, root_link: str, tip_link: str) -> PyKDL.Chain:
    """Build a KDL Chain from a URDF string between two links.

    Walks the URDF tree from tip_link back to root_link, then constructs
    the chain in root-to-tip order. Fixed joints are included as segments
    (they contribute mass but no DOF).

    Args:
        urdf_string: Complete URDF XML as a string.
        root_link: Name of the root link (chain base).
        tip_link: Name of the tip link (chain end-effector).

    Returns:
        PyKDL.Chain with segments from root to tip.

    Raises:
        ValueError: If no path exists between root and tip.
    """
    if isinstance(urdf_string, str):
        urdf_bytes = urdf_string.encode("utf-8")
    else:
        urdf_bytes = urdf_string
    robot = URDF.from_xml_string(urdf_bytes)

    # Build child→(joint, parent) map
    parent_map: dict[str, tuple] = {}
    for joint in robot.joints:
        parent_map[joint.child] = (joint, joint.parent)

    # Walk from tip to root to find the path
    path = []
    current = tip_link
    while current != root_link:
        if current not in parent_map:
            raise ValueError(
                f"No path from '{tip_link}' to '{root_link}': "
                f"link '{current}' has no parent joint."
            )
        joint, parent = parent_map[current]
        path.append((joint, current))
        current = parent

    path.reverse()  # root→tip order

    link_map = {link.name: link for link in robot.links}

    chain = PyKDL.Chain()
    for joint, child_link_name in path:
        kdl_joint = _joint_to_kdl(joint)
        kdl_frame = _pose_to_frame(joint.origin)

        child_link = link_map.get(child_link_name)
        kdl_inertia = _inertial_to_kdl(
            child_link.inertial if child_link else None
        )

        segment = PyKDL.Segment(child_link_name, kdl_joint, kdl_frame, kdl_inertia)
        chain.addSegment(segment)

    return chain


def mass_matrix_diagonal(
    chain: PyKDL.Chain,
    joint_positions: list[float],
) -> list[float]:
    """Compute the diagonal of the joint-space mass matrix H(q).

    H[i,i] gives the effective inertia seen by joint i at configuration q,
    accounting for all downstream link masses and the parallel axis theorem.

    Args:
        chain: KDL Chain (from build_chain).
        joint_positions: Joint angles in radians (length = chain DOFs).

    Returns:
        List of H[i,i] values for each joint DOF.
    """
    n = chain.getNrOfJoints()
    if len(joint_positions) != n:
        raise ValueError(
            f"Expected {n} joint positions, got {len(joint_positions)}"
        )

    solver = PyKDL.ChainDynParam(chain, PyKDL.Vector(0, 0, -9.81))

    q = PyKDL.JntArray(n)
    for i in range(n):
        q[i] = joint_positions[i]

    H = PyKDL.JntSpaceInertiaMatrix(n)
    ret = solver.JntToMass(q, H)
    if ret != 0:
        raise RuntimeError(f"JntToMass failed with code {ret}")

    return [H[i, i] for i in range(n)]
