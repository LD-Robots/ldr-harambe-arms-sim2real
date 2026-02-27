#!/usr/bin/env python3
"""
Isaac Sim 6.0 standalone launcher for the dual arm robot.

Automatically:
  - Imports the robot URDF into Isaac Sim
  - Sets up a physics scene (ground plane, lighting)
  - Creates the ROS2 OmniGraph bridge (joint states + clock)
  - Starts the simulation

After this script is running, launch the ROS2 side in a separate terminal:
    ros2 launch dual_arm_isaac isaac_sim.launch.py
Then MoveIt:
    ros2 launch dual_arm_moveit_config move_group.launch.py

Prerequisites:
    - Isaac Sim 6.0 built from source (~/isaacsim)
    - ROS2 Jazzy workspace built and sourced

Usage (recommended — use the wrapper script):
    ./run_isaac_sim.sh

  Or manually:
    # 1. Generate URDF with system Python (before Isaac Sim)
    source ~/colcon_ws/install/setup.bash
    python3 generate_urdf.py --output /tmp/dual_arm_isaac.urdf

    # 2. Run with Isaac Sim's Python
    ~/isaacsim/_build/linux-x86_64/release/python.sh launch_isaac_sim.py \\
        --urdf-path /tmp/dual_arm_isaac.urdf

    # Options:
    #   --headless          Run without GUI
    #   --save-usd FILE     Save configured scene to USD for faster future loads
    #   --load-usd FILE     Load pre-configured USD instead of importing URDF
"""

import argparse
import os
import sys


# ---------------------------------------------------------------------------
# Mimic joint definitions (from Inspire Hand xacro).
# Isaac Sim does NOT support URDF mimic joints natively, so we replicate the
# behaviour each physics step: target = parent_position * multiplier + offset.
# ---------------------------------------------------------------------------
MIMIC_JOINTS = {
    # mimic_joint: (parent_joint, multiplier, offset)
    # Left hand
    "left_thumb_intermediate_joint":  ("left_thumb_proximal_pitch_joint", 1.334,   0.0),
    "left_thumb_distal_joint":        ("left_thumb_proximal_pitch_joint", 0.667,   0.0),
    "left_index_intermediate_joint":  ("left_index_proximal_joint",      1.06399, -0.04545),
    "left_middle_intermediate_joint": ("left_middle_proximal_joint",     1.06399, -0.04545),
    "left_ring_intermediate_joint":   ("left_ring_proximal_joint",       1.06399, -0.04545),
    "left_pinky_intermediate_joint":  ("left_pinky_proximal_joint",      1.06399, -0.04545),
    # Right hand
    "right_thumb_intermediate_joint":  ("right_thumb_proximal_pitch_joint", 1.334,   0.0),
    "right_thumb_distal_joint":        ("right_thumb_proximal_pitch_joint", 0.667,   0.0),
    "right_index_intermediate_joint":  ("right_index_proximal_joint",      1.06399, -0.04545),
    "right_middle_intermediate_joint": ("right_middle_proximal_joint",     1.06399, -0.04545),
    "right_ring_intermediate_joint":   ("right_ring_proximal_joint",       1.06399, -0.04545),
    "right_pinky_intermediate_joint":  ("right_pinky_proximal_joint",      1.06399, -0.04545),
}


def _build_mimic_index_map(joint_names):
    """Build a DOF-index-based mimic map from the articulation's joint names.

    Returns:
        dict mapping parent_dof_index -> list of (mimic_dof_index, multiplier, offset)
    """
    name_to_idx = {name: i for i, name in enumerate(joint_names)}
    mimic_map = {}  # parent_idx -> [(mimic_idx, mult, offset), ...]

    for mimic_name, (parent_name, mult, offset) in MIMIC_JOINTS.items():
        mimic_idx = name_to_idx.get(mimic_name)
        parent_idx = name_to_idx.get(parent_name)
        if mimic_idx is not None and parent_idx is not None:
            mimic_map.setdefault(parent_idx, []).append((mimic_idx, mult, offset))
        elif mimic_idx is None and parent_idx is not None:
            print(f"[WARN] Mimic joint not found in articulation: {mimic_name}")
        # If parent not found either, the hand may not be present — skip silently

    total = sum(len(v) for v in mimic_map.values())
    print(f"[INFO] Mimic joint map: {total} mimic joints tracking "
          f"{len(mimic_map)} parent joints")
    return mimic_map


# ---------------------------------------------------------------------------
# Isaac Sim launcher (imports only available after SimulationApp)
# ---------------------------------------------------------------------------

def setup_ros2_bridge(robot_prim_path):
    """Create the OmniGraph that bridges Isaac Sim <-> ROS2.

    Publishes:  /isaac_joint_states  (sensor_msgs/JointState)
                /clock               (rosgraph_msgs/Clock)
    Subscribes: /isaac_joint_commands (sensor_msgs/JointState)
    """
    import omni.graph.core as og

    keys = og.Controller.Keys

    (graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": "/ROS2Bridge", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("PublishJointState.inputs:topicName", "/isaac_joint_states"),
                ("SubscribeJointState.inputs:topicName", "/isaac_joint_commands"),
                ("PublishClock.inputs:topicName", "/clock"),
                ("ArticulationController.inputs:robotPath", robot_prim_path),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
        },
    )

    # Set targetPrim as a USD relationship (not a simple string value)
    import omni.usd
    from pxr import Sdf
    stage = omni.usd.get_context().get_stage()

    for node_name in ["PublishJointState", "SubscribeJointState"]:
        node_prim = stage.GetPrimAtPath(f"/ROS2Bridge/{node_name}")
        if node_prim.IsValid():
            rel = node_prim.GetRelationship("inputs:targetPrim")
            if rel:
                rel.SetTargets([Sdf.Path(robot_prim_path)])
                print(f"[INFO] {node_name}.targetPrim → {robot_prim_path}")

    print("[INFO] ROS2 bridge OmniGraph created (/ROS2Bridge)")


def _find_articulation_root(stage, search_root):
    """Walk the prim tree under *search_root* and return the path of the
    first prim that carries the PhysicsArticulationRootAPI.

    The URDF importer typically applies this API to the first rigid-body
    link (e.g. ``base_link``), not to the top-level Xform container.
    """
    from pxr import UsdPhysics

    from pxr import Usd

    root_prim = stage.GetPrimAtPath(search_root)
    if not root_prim.IsValid():
        return None

    # Check direct children first (fast path — base_link is usually here)
    for prim in root_prim.GetChildren():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())

    # Full subtree search as fallback
    for prim in Usd.PrimRange(root_prim):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())

    return None


def _zero_hand_joints(stage, robot_root):
    """Set all hand/finger joint drive targets to 0 in the USD stage.

    This sets the drive target attributes so that even after world.reset()
    the joints will be driven to 0.
    """
    from pxr import Usd

    HAND_KEYWORDS = ("thumb", "index", "middle", "ring", "pinky")

    root_prim = stage.GetPrimAtPath(robot_root)
    if not root_prim.IsValid():
        return

    # Find joints by checking for drive:angular:physics:targetPosition attribute
    # (more reliable than prim.IsA(RevoluteJoint) across Isaac Sim versions)
    count = 0
    for prim in Usd.PrimRange(root_prim):
        target_attr = prim.GetAttribute("drive:angular:physics:targetPosition")
        if not target_attr or not target_attr.IsValid():
            continue

        path_str = str(prim.GetPath())
        path_lower = path_str.lower()
        is_hand = any(kw in path_lower for kw in HAND_KEYWORDS)
        old_val = target_attr.Get()

        # Configure drive parameters for ALL joints (arm + hand).
        # Isaac Sim uses acceleration-mode drives: torque = I × kp × error.
        # URDF effort limits map to maxForce which caps the drive torque.
        stiff_attr = prim.GetAttribute("drive:angular:physics:stiffness")
        damp_attr = prim.GetAttribute("drive:angular:physics:damping")
        max_force_attr = prim.GetAttribute("drive:angular:physics:maxForce")
        type_attr = prim.GetAttribute("drive:angular:physics:type")
        drive_type = type_attr.Get() if (type_attr and type_attr.IsValid()) else "N/A"

        if is_hand:
            target_attr.Set(0.0)
            state_attr = prim.GetAttribute("state:angular:physics:position")
            if state_attr and state_attr.IsValid():
                state_attr.Set(0.0)
            # Use acceleration-mode drives (same as arm joints).
            # With boosted inertia (1e-3 kg·m² from generate_urdf.py) and
            # reduced friction (0.5), these drives produce sufficient torque:
            # torque = inertia × kp × error = 1e-3 × 1e4 × error = 10 × error
            if stiff_attr and stiff_attr.IsValid():
                stiff_attr.Set(1e4)
            if damp_attr and damp_attr.IsValid():
                damp_attr.Set(1e3)
            if max_force_attr and max_force_attr.IsValid():
                max_force_attr.Set(1e4)
            count += 1
            print(f"  [HAND JOINT] {path_str}: target=0, kp=1e4, kd=1e3, maxF=1e4, type={drive_type}")
        else:
            # Arm joints: boost maxForce so drives aren't clamped by
            # the low URDF effort limits (real motors are 2-10 Nm,
            # but acceleration drives need much more headroom).
            if stiff_attr and stiff_attr.IsValid():
                stiff_attr.Set(1e4)
            if damp_attr and damp_attr.IsValid():
                damp_attr.Set(1e3)
            if max_force_attr and max_force_attr.IsValid():
                max_force_attr.Set(1e4)
            print(f"  [ARM  JOINT] {path_str}: target={old_val}, kp=1e4, kd=1e3, maxF=1e4, type={drive_type}")

    print(f"[INFO] Zeroed {count} hand joint drive targets")

    # Diagnostic: check inertia on hand links
    for prim in Usd.PrimRange(root_prim):
        path_lower = str(prim.GetPath()).lower()
        if not any(kw in path_lower for kw in HAND_KEYWORDS):
            continue
        mass_attr = prim.GetAttribute("physics:mass")
        inertia_attr = prim.GetAttribute("physics:diagonalInertia")
        if mass_attr and mass_attr.IsValid():
            m = mass_attr.Get()
            I = inertia_attr.Get() if (inertia_attr and inertia_attr.IsValid()) else None
            if m is not None and m > 0:
                print(f"  [LINK INERTIA] {prim.GetPath()}: mass={m:.6f}, inertia={I}")


def _zero_hand_joints_runtime(robot_root="/dual_arm_description"):
    """Zero all hand joint positions using the runtime physics API.

    Must be called AFTER world.reset() so the physics backend is initialized.
    Tries multiple APIs in order of preference for Isaac Sim 6.0 compatibility.

    Returns:
        (controller, hold_action, articulation) — articulation is the
        Articulation object for reading/writing joint state each frame.
    """
    import numpy as np

    HAND_KEYWORDS = ("thumb", "index", "middle", "ring", "pinky")

    # --- Try 1: High-level Articulation API (try several class names) ---
    ArticulationClass = None
    for import_path in [
        ("isaacsim.core.api.articulations", "Articulation"),
        ("isaacsim.core.api.articulations", "SingleArticulation"),
        ("isaacsim.core.prims", "SingleArticulation"),
        ("omni.isaac.core.articulations", "Articulation"),
    ]:
        try:
            mod = __import__(import_path[0], fromlist=[import_path[1]])
            ArticulationClass = getattr(mod, import_path[1])
            print(f"[INFO] Using {import_path[0]}.{import_path[1]}")
            break
        except (ImportError, AttributeError):
            continue

    if ArticulationClass is not None:
        try:
            artic = ArticulationClass(prim_path=robot_root)
            artic.initialize()
            joint_names = artic.dof_names
            positions = artic.get_joint_positions()

            if joint_names is not None and positions is not None:
                print(f"[INFO] Articulation has {len(joint_names)} DOFs")
                count = 0
                for i, name in enumerate(joint_names):
                    is_hand = any(kw in name.lower() for kw in HAND_KEYWORDS)
                    if is_hand:
                        print(f"  [HAND] {name}: {np.degrees(positions[i]):.1f}° → 0°")
                        positions[i] = 0.0
                        count += 1
                    else:
                        print(f"  [ARM ] {name}: {np.degrees(positions[i]):.1f}°")
                if count:
                    artic.set_joint_positions(positions)

                    # Set drive targets + gains via ArticulationController
                    try:
                        controller = artic.get_articulation_controller()
                        n_dof = len(joint_names)
                        # Build per-DOF stiffness/damping arrays
                        kps = np.zeros(n_dof)
                        kds = np.zeros(n_dof)
                        for i, name in enumerate(joint_names):
                            # All joints use acceleration-mode drives with same gains.
                            # Hand joints have boosted inertia (1e-3) so this produces
                            # enough torque to overcome friction and gravity.
                            kps[i] = 1e4
                            kds[i] = 1e3
                        controller.set_gains(kps, kds)
                        # Apply position action to set drive targets
                        from isaacsim.core.utils.types import ArticulationAction
                        controller.apply_action(
                            ArticulationAction(joint_positions=positions)
                        )
                        print(f"[INFO] Zeroed {count} hand joints (positions + drive targets + gains)")
                        return controller, ArticulationAction(joint_positions=positions), artic
                    except Exception as e2:
                        print(f"[WARN] ArticulationController failed: {e2}")
                        print(f"[INFO] Zeroed {count} hand joint positions only")
                return None, None, artic
        except Exception as e:
            print(f"[WARN] Articulation API failed: {e}")

    # --- Try 2: PhysX tensor API ---
    try:
        from omni.physics.tensors import create_simulation_view
        sim_view = create_simulation_view("numpy")
        artic_view = sim_view.create_articulation_view(robot_root)
        dof_names = artic_view.dof_names
        positions = artic_view.get_dof_positions().flatten()

        print(f"[INFO] Tensor API: {len(dof_names)} DOFs")
        count = 0
        for i, name in enumerate(dof_names):
            if any(kw in name.lower() for kw in HAND_KEYWORDS):
                print(f"  [HAND] {name}: {np.degrees(positions[i]):.1f}° → 0°")
                positions[i] = 0.0
                count += 1
        if count:
            new_pos = positions.reshape(1, -1)
            artic_view.set_dof_positions(new_pos)
            print(f"[INFO] Zeroed {count} hand joints via tensor API")
        return None, None, None
    except Exception as e:
        print(f"[WARN] Tensor API failed: {e}")

    # --- Try 3: List available articulation API names for debugging ---
    try:
        import isaacsim.core.api.articulations as artic_mod
        available = [x for x in dir(artic_mod) if not x.startswith("_")]
        print(f"[DEBUG] Available in isaacsim.core.api.articulations: {available}")
    except ImportError:
        pass
    print("[ERROR] Could not zero hand joints — no working API found")
    return None, None, None


def import_urdf(urdf_path):
    """Import the URDF into the current stage and return the prim path."""
    from isaacsim.asset.importer.urdf import _urdf

    urdf_interface = _urdf.acquire_urdf_interface()

    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.fix_base = True
    import_config.import_inertia_tensor = True
    import_config.distance_scale = 1.0
    import_config.density = 0.0
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.default_drive_strength = 1e4
    import_config.default_position_drive_damping = 1e3
    import_config.create_physics_scene = True

    # parse_urdf(root_dir, filename, config) — it constructs root_dir/filename
    dest_path = "/World/dual_arm"
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    urdf_filename = os.path.basename(urdf_path)

    print(f"[INFO] Parsing URDF: {urdf_dir}/{urdf_filename}")
    parsed_urdf = urdf_interface.parse_urdf(urdf_dir, urdf_filename, import_config)

    prim_path = urdf_interface.import_robot(
        urdf_dir, urdf_filename, parsed_urdf, import_config, dest_path
    )

    # Isaac Sim creates the robot at /<robot_name> (from URDF),
    # ignoring dest_path for in-memory stages.
    # The URDF robot name is "dual_arm_description".
    import omni.usd
    from pxr import UsdGeom, UsdPhysics, Gf
    stage = omni.usd.get_context().get_stage()

    # Use the path from import_robot, or find it by known name
    robot_root = prim_path
    if not robot_root or not stage.GetPrimAtPath(robot_root).IsValid():
        robot_root = "/dual_arm_description"
    if not stage.GetPrimAtPath(robot_root).IsValid():
        robot_root = dest_path

    print(f"[INFO] Robot root Xform: {robot_root}")

    # Raise the robot above the ground plane (z=1.0, same as Gazebo)
    robot_prim = stage.GetPrimAtPath(robot_root)
    if robot_prim.IsValid():
        xform = UsdGeom.Xformable(robot_prim)
        # Preserve any existing xform ops — just add a translate
        existing_ops = xform.GetOrderedXformOps()
        if existing_ops:
            # Check if there's already a translate op we can modify
            for op in existing_ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(0.0, 0.0, 1.26))
                    break
            else:
                xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
                    Gf.Vec3d(0.0, 0.0, 1.26))
        else:
            xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 1.26))
        print("[INFO] Robot raised to z=1.26")

    # Find the actual articulation root (usually on base_link, not the Xform)
    artic_path = _find_articulation_root(stage, robot_root)
    if artic_path:
        print(f"[INFO] Articulation root found: {artic_path}")
    else:
        # Fallback: apply ArticulationRootAPI on the robot root
        print(f"[WARN] No ArticulationRootAPI found, applying to {robot_root}")
        UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
        artic_path = robot_root

    # Set all hand joints to 0° drive targets
    _zero_hand_joints(stage, robot_root)

    # Print the prim hierarchy for diagnostics
    print("[INFO] Robot prim hierarchy (top-level children):")
    for child in robot_prim.GetChildren():
        has_artic = child.HasAPI(UsdPhysics.ArticulationRootAPI)
        tag = " [ARTICULATION_ROOT]" if has_artic else ""
        print(f"  {child.GetPath()}{tag}")

    return artic_path


def setup_scene():
    """Add ground plane and dome light."""
    from isaacsim.core.api import World

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Add dome light for environment illumination
    import omni.kit.commands
    omni.kit.commands.execute(
        "CreatePrim",
        prim_path="/World/DomeLight",
        prim_type="DomeLight",
        attributes={"inputs:intensity": 1000.0},
    )

    return world


def setup_camera(stage, robot_root):
    """Create an RGBD camera sensor on camera_depth_frame.

    Matches the Orbbec Gemini 336L specifications:
      - 90° horizontal FOV
      - 1280×800 resolution
      - 0.17 m – 20 m depth range

    Returns the camera prim path, or None if camera_depth_frame not found.
    """
    from pxr import UsdGeom, Gf, Usd

    # Find camera_depth_frame in the robot hierarchy
    root_prim = stage.GetPrimAtPath(robot_root)
    if not root_prim.IsValid():
        print(f"[WARN] Robot root not found: {robot_root}, skipping camera setup")
        return None

    camera_frame_path = None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == "camera_depth_frame":
            camera_frame_path = str(prim.GetPath())
            break

    if not camera_frame_path:
        print("[WARN] camera_depth_frame not found in robot, skipping camera setup")
        return None

    # Create Camera prim as child of the depth frame
    camera_prim_path = f"{camera_frame_path}/CameraSensor"
    camera = UsdGeom.Camera.Define(stage, camera_prim_path)

    # Gemini 336L: 90° HFOV at 1280×800 (16:10 aspect ratio)
    # USD camera model: HFOV = 2 × atan(horizontalAperture / (2 × focalLength))
    # For 90° HFOV: focalLength = horizontalAperture / 2
    horizontal_aperture = 20.0  # mm
    vertical_aperture = horizontal_aperture * 800.0 / 1280.0  # 12.5 mm
    focal_length = horizontal_aperture / 2.0  # 10 mm → 90° HFOV

    camera.GetHorizontalApertureAttr().Set(horizontal_aperture)
    camera.GetVerticalApertureAttr().Set(vertical_aperture)
    camera.GetFocalLengthAttr().Set(focal_length)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.17, 20.0))

    # Orient camera: URDF depth frame has X-forward, Z-up (body convention).
    # Isaac Sim Camera looks along local -Z with +Y up.
    # Rotation (90°, 0°, -90°) XYZ maps: cam -Z → frame +X, cam +Y → frame +Z
    xformable = UsdGeom.Xformable(camera.GetPrim())
    xformable.AddRotateXYZOp().Set(Gf.Vec3f(90.0, 0.0, -90.0))

    print(f"[INFO] Camera sensor created: {camera_prim_path}")
    print(f"  Specs: 90° HFOV, 1280x800, depth range 0.17-20.0m")
    return camera_prim_path


def setup_camera_ros2_bridge(camera_prim_path):
    """Add RGBD camera publishers to the existing ROS2 OmniGraph.

    Creates a render product and ROS2CameraHelper nodes that publish:
      - /camera/color/image_raw   (RGB)
      - /camera/depth/image_raw   (depth)
      - /camera/color/camera_info (camera intrinsics)
      - /camera/depth/camera_info (camera intrinsics)
      - /camera/depth/points      (point cloud)
    """
    import omni.graph.core as og
    import omni.replicator.core as rep

    # Create render product (binds camera to a 1280×800 render output)
    render_product = rep.create.render_product(camera_prim_path, (1280, 800))
    rp_path = render_product.path

    keys = og.Controller.Keys

    # Add camera helper nodes to the existing /ROS2Bridge graph.
    # ROS2CameraHelper types: rgb, depth, depth_pcl (no camera_info — use ROS2CameraInfoHelper)
    og.Controller.edit(
        "/ROS2Bridge",
        {
            keys.CREATE_NODES: [
                ("RGBCameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthCameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PointCloudHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfoHelper", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                ("RGBCameraHelper.inputs:type", "rgb"),
                ("RGBCameraHelper.inputs:topicName", "/camera/color/image_raw"),
                ("RGBCameraHelper.inputs:frameId", "camera_color_optical_frame"),
                ("RGBCameraHelper.inputs:renderProductPath", rp_path),

                ("DepthCameraHelper.inputs:type", "depth"),
                ("DepthCameraHelper.inputs:topicName", "/camera/depth/image_raw"),
                ("DepthCameraHelper.inputs:frameId", "camera_depth_frame"),
                ("DepthCameraHelper.inputs:renderProductPath", rp_path),

                ("PointCloudHelper.inputs:type", "depth_pcl"),
                ("PointCloudHelper.inputs:topicName", "/camera/depth/points"),
                ("PointCloudHelper.inputs:frameId", "camera_depth_frame"),
                ("PointCloudHelper.inputs:renderProductPath", rp_path),

                # CameraInfoHelper is a separate node type (not ROS2CameraHelper)
                ("CameraInfoHelper.inputs:topicName", "/camera/color/camera_info"),
                ("CameraInfoHelper.inputs:frameId", "camera_color_optical_frame"),
                ("CameraInfoHelper.inputs:renderProductPath", rp_path),
            ],
            # Use full paths for nodes created in the initial setup_ros2_bridge() call
            keys.CONNECT: [
                ("/ROS2Bridge/OnPlaybackTick.outputs:tick", "RGBCameraHelper.inputs:execIn"),
                ("/ROS2Bridge/OnPlaybackTick.outputs:tick", "DepthCameraHelper.inputs:execIn"),
                ("/ROS2Bridge/OnPlaybackTick.outputs:tick", "PointCloudHelper.inputs:execIn"),
                ("/ROS2Bridge/OnPlaybackTick.outputs:tick", "CameraInfoHelper.inputs:execIn"),
                ("/ROS2Bridge/Context.outputs:context", "RGBCameraHelper.inputs:context"),
                ("/ROS2Bridge/Context.outputs:context", "DepthCameraHelper.inputs:context"),
                ("/ROS2Bridge/Context.outputs:context", "PointCloudHelper.inputs:context"),
                ("/ROS2Bridge/Context.outputs:context", "CameraInfoHelper.inputs:context"),
            ],
        },
    )

    print("[INFO] Camera ROS2 bridge nodes added to /ROS2Bridge")
    print(f"  RGB:         /camera/color/image_raw")
    print(f"  Depth:       /camera/depth/image_raw")
    print(f"  CameraInfo:  /camera/color/camera_info")
    print(f"  PointCloud:  /camera/depth/points")


def main():
    parser = argparse.ArgumentParser(
        description="Isaac Sim 6.0 launcher for dual arm robot"
    )
    parser.add_argument(
        "--urdf-path", type=str, required=False,
        help="Path to pre-generated URDF with absolute mesh paths",
    )
    parser.add_argument(
        "--headless", action="store_true", help="Run without GUI"
    )
    parser.add_argument(
        "--save-usd", type=str, default=None,
        help="Save the configured scene to a USD file",
    )
    parser.add_argument(
        "--load-usd", type=str, default=None,
        help="Load a pre-configured USD scene (skip URDF import)",
    )
    args = parser.parse_args()

    # Validate: need either --urdf-path or --load-usd
    urdf_path = args.urdf_path
    if not args.load_usd and not urdf_path:
        sys.exit(
            "[ERROR] Provide --urdf-path or --load-usd.\n"
            "  Use run_isaac_sim.sh to handle URDF generation automatically."
        )
    if urdf_path and not os.path.isfile(urdf_path):
        sys.exit(f"[ERROR] URDF file not found: {urdf_path}")

    # --- Start Isaac Sim ---
    from isaacsim import SimulationApp

    config = {
        "headless": args.headless,
        "width": 1920,
        "height": 1080,
        "anti_aliasing": 0,
    }
    simulation_app = SimulationApp(config)
    print("[INFO] Isaac Sim started")

    # Enable the ROS2 bridge extension
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    # Enable URDF importer extension
    enable_extension("isaacsim.asset.importer.urdf")
    simulation_app.update()

    # --- Set up scene ---
    camera_prim_path = None

    if args.load_usd:
        import omni.usd
        omni.usd.get_context().open_stage(args.load_usd)
        simulation_app.update()
        print(f"[INFO] Loaded USD scene: {args.load_usd}")
        from isaacsim.core.api import World
        world = World(stage_units_in_meters=1.0)
        # Find the articulation root in the loaded USD
        stage = omni.usd.get_context().get_stage()
        robot_prim_path = (
            _find_articulation_root(stage, "/dual_arm_description")
            or _find_articulation_root(stage, "/World/dual_arm")
            or "/dual_arm_description"
        )
        print(f"[INFO] Articulation root: {robot_prim_path}")

        # Find existing camera sensor in loaded USD (saved from previous run)
        from pxr import Usd, UsdGeom
        for prim in Usd.PrimRange(stage.GetPrimAtPath("/dual_arm_description")):
            if prim.GetName() == "CameraSensor" and prim.IsA(UsdGeom.Camera):
                camera_prim_path = str(prim.GetPath())
                print(f"[INFO] Found camera in USD: {camera_prim_path}")
                break
    else:
        world = setup_scene()
        simulation_app.update()

        # Import URDF
        robot_prim_path = import_urdf(urdf_path)
        simulation_app.update()

        # Create RGBD camera sensor on camera_depth_frame
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        camera_prim_path = setup_camera(stage, "/dual_arm_description")
        simulation_app.update()

        # Clean up temp URDF
        os.unlink(urdf_path)

    # --- Save USD if requested ---
    if args.save_usd:
        import omni.usd
        omni.usd.get_context().save_as_stage(args.save_usd, None)
        print(f"[INFO] Scene saved to: {args.save_usd}")

    # --- Initialize physics and settle joints BEFORE creating ROS2 bridge ---
    # The OmniGraph ArticulationController can override drive settings when
    # receiving empty commands, so we let joints settle first.

    import omni.usd

    world.reset()

    # Re-apply hand joint drive settings after world.reset() — it resets
    # USD drive attributes to import defaults.
    stage = omni.usd.get_context().get_stage()
    _zero_hand_joints(stage, "/dual_arm_description")

    simulation_app.update()

    # Zero hand joint positions using the runtime Articulation API.
    hold_controller, hold_action, articulation = _zero_hand_joints_runtime()

    # Build mimic joint index map for runtime replication
    import numpy as np
    mimic_map = {}
    if articulation is not None:
        joint_names = articulation.dof_names
        if joint_names is not None:
            mimic_map = _build_mimic_index_map(joint_names)

    # Cache controller and action type for the mimic loop
    _mimic_controller = None
    _ArticulationAction = None
    if mimic_map and articulation is not None:
        try:
            _mimic_controller = articulation.get_articulation_controller()
            from isaacsim.core.utils.types import ArticulationAction as _AA
            _ArticulationAction = _AA
        except Exception as e:
            print(f"[WARN] Could not get controller for mimic joints: {e}")

    def _apply_mimic_targets():
        """Read proximal positions, compute mimic targets, apply them."""
        if not mimic_map or _mimic_controller is None:
            return
        positions = articulation.get_joint_positions()
        if positions is None:
            return
        mimic_indices = []
        mimic_targets = []
        for parent_idx, mimics in mimic_map.items():
            parent_pos = positions[parent_idx]
            for mimic_idx, mult, offset in mimics:
                mimic_indices.append(mimic_idx)
                mimic_targets.append(parent_pos * mult + offset)
        _mimic_controller.apply_action(
            _ArticulationAction(
                joint_positions=np.array(mimic_targets),
                joint_indices=np.array(mimic_indices),
            )
        )

    # Settle: run physics steps while continuously applying the hold action.
    SETTLE_STEPS = 50
    print(f"[INFO] Settling joints for {SETTLE_STEPS} steps...")
    for _ in range(SETTLE_STEPS):
        if hold_controller and hold_action:
            hold_controller.apply_action(hold_action)
        _apply_mimic_targets()
        world.step(render=True)
    print("[INFO] Joint settling complete")

    # --- NOW create ROS2 bridge (after joints are stable) ---
    setup_ros2_bridge(robot_prim_path)
    simulation_app.update()

    # --- Set up camera ROS2 publishers ---
    if camera_prim_path:
        setup_camera_ros2_bridge(camera_prim_path)
        simulation_app.update()

    # --- Run simulation ---
    print("[INFO] Simulation running. Press Ctrl+C to stop.")
    print("[INFO] Now launch the ROS2 side in another terminal:")
    print("         ros2 launch dual_arm_isaac isaac_sim.launch.py")
    print("       Then MoveIt:")
    print("         ros2 launch dual_arm_moveit_config move_group.launch.py")

    try:
        while simulation_app.is_running():
            # The OmniGraph ROS2 bridge owns arm + hand proximal commands.
            # We replicate mimic joints (intermediate/distal) each frame
            # since Isaac Sim doesn't support URDF mimic tags natively.
            _apply_mimic_targets()
            world.step(render=True)
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down...")

    simulation_app.close()


if __name__ == "__main__":
    main()
