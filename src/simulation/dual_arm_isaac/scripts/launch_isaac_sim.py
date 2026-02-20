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
                    op.Set(Gf.Vec3d(0.0, 0.0, 1.0))
                    break
            else:
                xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
                    Gf.Vec3d(0.0, 0.0, 1.0))
        else:
            xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 1.0))
        print("[INFO] Robot raised to z=1.0")

    # Find the actual articulation root (usually on base_link, not the Xform)
    artic_path = _find_articulation_root(stage, robot_root)
    if artic_path:
        print(f"[INFO] Articulation root found: {artic_path}")
    else:
        # Fallback: apply ArticulationRootAPI on the robot root
        print(f"[WARN] No ArticulationRootAPI found, applying to {robot_root}")
        UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
        artic_path = robot_root

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
    else:
        world = setup_scene()
        simulation_app.update()

        # Import URDF
        robot_prim_path = import_urdf(urdf_path)
        simulation_app.update()

        # Clean up temp URDF
        os.unlink(urdf_path)

    # --- Create ROS2 bridge ---
    setup_ros2_bridge(robot_prim_path)
    simulation_app.update()

    # --- Save USD if requested ---
    if args.save_usd:
        import omni.usd
        omni.usd.get_context().save_as_stage(args.save_usd, None)
        print(f"[INFO] Scene saved to: {args.save_usd}")

    # --- Run simulation ---
    print("[INFO] Simulation running. Press Ctrl+C to stop.")
    print("[INFO] Now launch the ROS2 side in another terminal:")
    print("         ros2 launch dual_arm_isaac isaac_sim.launch.py")
    print("       Then MoveIt:")
    print("         ros2 launch dual_arm_moveit_config move_group.launch.py")

    world.reset()

    try:
        while simulation_app.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down...")

    simulation_app.close()


if __name__ == "__main__":
    main()
