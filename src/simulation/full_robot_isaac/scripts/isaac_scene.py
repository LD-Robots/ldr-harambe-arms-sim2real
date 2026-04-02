#!/usr/bin/env python3
"""
Isaac Lab standalone scene for Harambe humanoid robot.

Launches Isaac Sim with the robot on a ground plane.
Optionally runs an ONNX policy for locomotion.

Usage:
    # Just view the robot (no policy):
    /home/alex/IsaacLab/isaaclab.sh -p scripts/isaac_scene.py

    # With policy:
    /home/alex/IsaacLab/isaaclab.sh -p scripts/isaac_scene.py \
        --onnx /path/to/policy.onnx

    # With ROS 2 bridge (publishes joint states, receives commands):
    /home/alex/IsaacLab/isaaclab.sh -p scripts/isaac_scene.py --ros2
"""

import argparse
import os
import sys

# Match ROS 2 DDS config (must be set before any ROS/Isaac import)
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
os.environ.setdefault("ROS_DOMAIN_ID", "42")
cyclone_cfg = os.path.expanduser("~/cyclonedds.xml")
if os.path.exists(cyclone_cfg):
    os.environ.setdefault("CYCLONEDDS_URI", f"file://{cyclone_cfg}")

from isaaclab.app import AppLauncher


def _str2bool(v):
    if isinstance(v, bool):
        return v
    return str(v).strip().lower() in ("1", "true", "yes", "on")


parser = argparse.ArgumentParser(description="Isaac Lab scene for Harambe robot")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--onnx", type=str, default="", help="Path to ONNX policy")
parser.add_argument("--cmd_vel_x", type=float, default=0.0, help="Forward velocity command")
parser.add_argument("--cmd_vel_y", type=float, default=0.0, help="Lateral velocity command")
parser.add_argument("--cmd_yaw", type=float, default=0.0, help="Yaw rate command")
parser.add_argument(
    "--obs_from_imu",
    type=_str2bool,
    nargs="?",
    const=True,
    default=False,
    help="Build policy obs from IMU-equivalent transforms (matches external deploy path)",
)
parser.add_argument(
    "--log_imu_obs",
    type=_str2bool,
    nargs="?",
    const=True,
    default=False,
    help="Log IMU-related ONNX obs terms and compare imu-equivalent vs isaac-direct",
)
parser.add_argument(
    "--log_imu_obs_every",
    type=int,
    default=100,
    help="Periodic IMU obs logging interval in sim steps",
)
parser.add_argument("--ros2", action="store_true", help="Enable ROS 2 joint state/command bridge")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- After sim launch ---

import torch
import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sensors import ImuCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass

# URDF path
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.abspath(os.path.join(_SCRIPT_DIR, ".."))
_SRC_DIR = os.path.abspath(os.path.join(_PKG_DIR, "..", ".."))  # src/simulation/full_robot_isaac → src/
_URDF_PATH = os.path.join(
    _SRC_DIR, "robot_description", "full_robot_description", "urdf",
    "full_robot_isaaclab.urdf",
)

HARDWARE_JOINT_ORDER = [
    "left_shoulder_pitch_joint_X6", "left_shoulder_roll_joint_X6",
    "left_shoulder_yaw_joint_X4", "left_elbow_pitch_joint_X6",
    "left_wrist_yaw_joint_X4", "left_wrist_roll_joint_X4",
    "right_shoulder_pitch_joint_X6", "right_shoulder_roll_joint_X6",
    "right_shoulder_yaw_joint_X4", "right_elbow_pitch_joint_X6",
    "right_wrist_yaw_joint_X4", "right_wrist_roll_joint_X4",
    "waist_yaw_joint_X8",
    "left_hip_pitch_joint_X8", "left_hip_roll_joint_X8",
    "left_hip_yaw_joint_X8", "left_knee_joint_X8",
    "left_ankle_pitch_joint_X4", "left_ankle_roll_joint_X4",
    "right_hip_pitch_joint_X8", "right_hip_roll_joint_X8",
    "right_hip_yaw_joint_X8", "right_knee_joint_X8",
    "right_ankle_pitch_joint_X4", "right_ankle_roll_joint_X4",
]


def quat_rotate_inverse_xyzw(q_xyzw: torch.Tensor, v_xyz: torch.Tensor) -> torch.Tensor:
    """Rotate vector by inverse quaternion (q in x,y,z,w)."""
    q_vec = q_xyzw[:3]
    q_w = q_xyzw[3]
    a = v_xyz * (2.0 * q_w * q_w - 1.0)
    b = torch.cross(q_vec, v_xyz, dim=0) * q_w * 2.0
    c = q_vec * (torch.dot(q_vec, v_xyz) * 2.0)
    return a - b + c


def quat_rotate_inverse_wxyz(q_wxyz: torch.Tensor, v_xyz: torch.Tensor) -> torch.Tensor:
    """Rotate vector by inverse quaternion (q in w,x,y,z)."""
    return quat_rotate_inverse_xyzw(
        torch.stack([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]]), v_xyz
    )


def sensor_to_base(v_sensor: torch.Tensor) -> torch.Tensor:
    """Same transform as deploy_full_body_ros2.py."""
    return torch.stack([-v_sensor[2], -v_sensor[1], -v_sensor[0]])


def build_imu_equivalent_obs_terms(root_quat_wxyz: torch.Tensor, ang_vel_body: torch.Tensor):
    """Rebuild projected_gravity/base_ang_vel through the same IMU path used by external deploy."""
    # Isaac body-frame gravity from world down vector
    grav_body = quat_rotate_inverse_wxyz(
        root_quat_wxyz, torch.tensor([0.0, 0.0, -1.0], device=root_quat_wxyz.device)
    )

    # Bridge path: body -> sensor
    grav_sensor = torch.stack([-grav_body[2], -grav_body[1], -grav_body[0]])
    ang_vel_sensor = torch.stack([-ang_vel_body[2], -ang_vel_body[1], -ang_vel_body[0]])

    # Bridge publishes quaternion rotating [1,0,0] -> grav_sensor
    ref = torch.tensor([1.0, 0.0, 0.0], device=root_quat_wxyz.device)
    grav_sensor_n = grav_sensor / (torch.linalg.norm(grav_sensor) + 1e-8)
    dot = torch.clamp(torch.dot(ref, grav_sensor_n), -1.0, 1.0)
    cross = torch.cross(ref, grav_sensor_n, dim=0)
    cross_norm = torch.linalg.norm(cross)

    if cross_norm < 1e-6:
        if dot > 0:
            sim_q_xyzw = torch.tensor([0.0, 0.0, 0.0, 1.0], device=root_quat_wxyz.device)
        else:
            sim_q_xyzw = torch.tensor([0.0, 1.0, 0.0, 0.0], device=root_quat_wxyz.device)
    else:
        axis = cross / cross_norm
        angle = torch.arccos(dot)
        half = angle / 2.0
        s = torch.sin(half)
        sim_q_xyzw = torch.stack([axis[0] * s, axis[1] * s, axis[2] * s, torch.cos(half)])

    # Deploy path with use_gazebo_imu=true:
    # gravity_sensor = quat_rotate_inverse(q, [1,0,0]); proj_gravity = sensor_to_base(gravity_sensor)
    gravity_ref = torch.tensor([1.0, 0.0, 0.0], device=root_quat_wxyz.device)
    gravity_sensor_recovered = quat_rotate_inverse_xyzw(sim_q_xyzw, gravity_ref)
    projected_gravity = sensor_to_base(gravity_sensor_recovered)
    # Align IMU-equivalent gravity sign convention with Isaac direct projected_gravity_b.
    projected_gravity = torch.stack(
        [-projected_gravity[0], -projected_gravity[1], projected_gravity[2]]
    )
    base_ang_vel = sensor_to_base(ang_vel_sensor)
    return base_ang_vel, projected_gravity


@configclass
class HarambeSceneCfg(InteractiveSceneCfg):
    num_envs = 1
    env_spacing = 2.5

    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=1.0, dynamic_friction=1.0, restitution=0.0,
        ),
    )

    robot = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.UrdfFileCfg(
            asset_path=_URDF_PATH,
            force_usd_conversion=True,
            make_instanceable=True,
            fix_base=False,
            merge_fixed_joints=True,
            joint_drive=None,
            collider_type="convex_hull",
            self_collision=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=8,
            ),
            activate_contact_sensors=True,
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 1.25),
            joint_pos={
                ".*_hip_pitch_joint_X8": -0.15,
                ".*_hip_roll_joint_X8": 0.0,
                ".*_hip_yaw_joint_X8": 0.0,
                ".*_knee_joint_X8": 0.30,
                ".*_ankle_pitch_joint_X4": -0.15,
                ".*_ankle_roll_joint_X4": 0.0,
                "waist_yaw_joint_X8": 0.0,
                ".*_shoulder_pitch_joint_X6": 0.0,
                ".*_shoulder_roll_joint_X6": 0.0,
                ".*_shoulder_yaw_joint_X4": 0.0,
                ".*_elbow_pitch_joint_X6": 0.0,
                ".*_wrist_yaw_joint_X4": 0.0,
                ".*_wrist_roll_joint_X4": 0.0,
            },
            joint_vel={".*": 0.0},
        ),
        actuators={
            "legs": ImplicitActuatorCfg(
                joint_names_expr=[
                    "waist_yaw_joint_X8", ".*_hip_pitch_joint_X8",
                    ".*_hip_roll_joint_X8", ".*_hip_yaw_joint_X8",
                    ".*_knee_joint_X8",
                ],
                effort_limit_sim=100.0,
                velocity_limit_sim=13.3,
                stiffness={
                    "waist_yaw_joint_X8": 416.0,
                    ".*_hip_pitch_joint_X8": 1553.0,
                    ".*_hip_roll_joint_X8": 1548.0,
                    ".*_hip_yaw_joint_X8": 252.0,
                    ".*_knee_joint_X8": 324.0,
                },
                damping={
                    "waist_yaw_joint_X8": 24.0,
                    ".*_hip_pitch_joint_X8": 77.7,
                    ".*_hip_roll_joint_X8": 77.4,
                    ".*_hip_yaw_joint_X8": 12.6,
                    ".*_knee_joint_X8": 16.2,
                },
                armature=0.01,
                friction=3.21,
            ),
            "ankle_pitch": ImplicitActuatorCfg(
                joint_names_expr=[".*_ankle_pitch_joint_X4"],
                effort_limit_sim=96.6,
                velocity_limit_sim=2.17,
                stiffness=524.0,
                damping=26.2,
                armature=0.01,
                friction=1.14,
            ),
            "ankle_roll": ImplicitActuatorCfg(
                joint_names_expr=[".*_ankle_roll_joint_X4"],
                effort_limit_sim=28.98,
                velocity_limit_sim=7.24,
                stiffness=157.0,
                damping=7.9,
                armature=0.01,
                friction=1.14,
            ),
            "arms_X6": ImplicitActuatorCfg(
                joint_names_expr=[
                    ".*_shoulder_pitch_joint_X6", ".*_shoulder_roll_joint_X6",
                    ".*_elbow_pitch_joint_X6",
                ],
                effort_limit_sim=23.0,
                velocity_limit_sim=16.02,
                stiffness={
                    ".*_shoulder_pitch_joint_X6": 435.0,
                    ".*_shoulder_roll_joint_X6": 440.0,
                    ".*_elbow_pitch_joint_X6": 118.0,
                },
                damping={
                    ".*_shoulder_pitch_joint_X6": 29.0,
                    ".*_shoulder_roll_joint_X6": 29.3,
                    ".*_elbow_pitch_joint_X6": 7.9,
                },
                armature=0.01,
                friction=1.6,
            ),
            "arms_X4": ImplicitActuatorCfg(
                joint_names_expr=[
                    ".*_shoulder_yaw_joint_X4", ".*_wrist_yaw_joint_X4",
                    ".*_wrist_roll_joint_X4",
                ],
                effort_limit_sim=12.075,
                velocity_limit_sim=8.69,
                stiffness={
                    ".*_shoulder_yaw_joint_X4": 93.0,
                    ".*_wrist_yaw_joint_X4": 38.5,
                    ".*_wrist_roll_joint_X4": 45.3,
                },
                damping={
                    ".*_shoulder_yaw_joint_X4": 6.2,
                    ".*_wrist_yaw_joint_X4": 2.6,
                    ".*_wrist_roll_joint_X4": 3.0,
                },
                armature=0.01,
                friction=1.14,
            ),
        },
    )

    # IMU sensor on pelvis
    pelvis_imu = ImuCfg(
        prim_path="/World/envs/env_.*/Robot/urdf_base",
        update_period=0.01,  # 100 Hz
    )

    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(intensity=750.0),
    )


def main():
    # Simulation config
    sim_cfg = SimulationCfg(dt=0.005, render_interval=4)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[3.0, 3.0, 2.0], target=[0.0, 0.0, 1.0])

    # Create scene
    scene_cfg = HarambeSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.5)
    scene = InteractiveScene(scene_cfg)

    # Load ONNX policy if provided
    ort_session = None
    if args_cli.onnx and os.path.exists(args_cli.onnx):
        import onnxruntime as ort
        ort_session = ort.InferenceSession(args_cli.onnx, providers=["CPUExecutionProvider"])
        print(f"[INFO] Loaded ONNX policy: {args_cli.onnx}")
    else:
        print("[INFO] No policy — robot will hold default pose")

    # Reset
    sim.reset()
    scene.reset()

    robot = scene["robot"]
    print(f"[INFO] Robot: {len(robot.joint_names)} joints", flush=True)

    # Map first 25 joints (actuated) for external control
    actuated_joint_ids = list(range(25))

    # Bidirectional UDP bridge: Isaac ↔ ROS 2 deploy script
    import socket
    import struct
    import threading

    # Outgoing: joint_states + IMU → ROS 2
    udp_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    out_addr = ("127.0.0.1", 9870)
    out_addr2 = ("127.0.0.1", 9872)  # direct UDP deploy

    # Incoming: joint commands ← ROS 2
    udp_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_in.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_in.bind(("127.0.0.1", 9871))
    udp_in.setblocking(False)

    external_targets = None  # will be set when commands received
    external_control = False

    print("[INFO] UDP bridge: out=9870 (states+IMU), in=9871 (commands)")
    print(f"[INFO] Joint names: {robot.joint_names}")

    # Build joint order mapping: Isaac URDF order ↔ HARDWARE_JOINT_ORDER
    ISAAC_ORDER = [
        "left_hip_pitch_joint_X8", "right_hip_pitch_joint_X8", "waist_yaw_joint_X8",
        "left_hip_roll_joint_X8", "right_hip_roll_joint_X8",
        "left_shoulder_pitch_joint_X6", "right_shoulder_pitch_joint_X6",
        "left_hip_yaw_joint_X8", "right_hip_yaw_joint_X8",
        "left_shoulder_roll_joint_X6", "right_shoulder_roll_joint_X6",
        "left_knee_joint_X8", "right_knee_joint_X8",
        "left_shoulder_yaw_joint_X4", "right_shoulder_yaw_joint_X4",
        "left_ankle_pitch_joint_X4", "right_ankle_pitch_joint_X4",
        "left_elbow_pitch_joint_X6", "right_elbow_pitch_joint_X6",
        "left_ankle_roll_joint_X4", "right_ankle_roll_joint_X4",
        "left_wrist_yaw_joint_X4", "right_wrist_yaw_joint_X4",
        "left_wrist_roll_joint_X4", "right_wrist_roll_joint_X4",
    ]
    # isaac_to_hw[i] = Isaac index of HARDWARE_JOINT_ORDER[i]
    isaac_to_hw = torch.tensor([ISAAC_ORDER.index(name) for name in HARDWARE_JOINT_ORDER], device=sim.device)
    # hw_to_isaac[i] = HW index of ISAAC_ORDER[i]
    hw_to_isaac = torch.tensor([HARDWARE_JOINT_ORDER.index(name) for name in ISAAC_ORDER], device=sim.device)

    # State
    prev_action = torch.zeros(args_cli.num_envs, 25, device=sim.device)
    commands = torch.zeros(args_cli.num_envs, 3, device=sim.device)
    commands[0, 0] = args_cli.cmd_vel_x
    commands[0, 1] = args_cli.cmd_vel_y
    commands[0, 2] = args_cli.cmd_yaw
    step_count = 0
    print(f"[INFO] Commands: vx={args_cli.cmd_vel_x}, vy={args_cli.cmd_vel_y}, yaw={args_cli.cmd_yaw}", flush=True)
    print(f"[INFO] ONNX obs source: {'imu-equivalent' if args_cli.obs_from_imu else 'isaac-direct'}", flush=True)
    if args_cli.log_imu_obs:
        print(f"[INFO] IMU obs logging enabled: every {args_cli.log_imu_obs_every} steps", flush=True)

    # Default positions in HARDWARE_JOINT_ORDER
    default_pos_hw = torch.zeros(25, device=sim.device)
    default_pos_hw[13] = -0.15  # L hip_pitch
    default_pos_hw[16] = 0.30   # L knee
    default_pos_hw[17] = -0.15  # L ankle_pitch
    default_pos_hw[19] = -0.15  # R hip_pitch
    default_pos_hw[22] = 0.30   # R knee
    default_pos_hw[23] = -0.15  # R ankle_pitch

    print("[INFO] Running... Press Ctrl+C to stop.", flush=True)

    # Default targets in Isaac order (for holding pose)
    default_targets_isaac = default_pos_hw[hw_to_isaac]
    print(f"[DEBUG] default_targets_isaac = {default_targets_isaac.cpu().numpy().round(3).tolist()}", flush=True)

    while simulation_app.is_running():
        # --- 1. Receive external commands (UDP from deploy script) ---
        try:
            cmd_data, _ = udp_in.recvfrom(4096)
            if len(cmd_data) == 25 * 4:
                cmd_values = struct.unpack('25f', cmd_data)
                external_targets = torch.tensor(cmd_values, device=sim.device, dtype=torch.float32)
                external_control = True
        except BlockingIOError:
            pass

        # --- 2. Compute targets ---
        # External commands override internal ONNX policy
        if external_control and external_targets is not None:
            # Reset robot on first external command (handover from internal ONNX)
            if not hasattr(main, '_external_started'):
                main._external_started = True
                # Reset to init state
                robot.write_root_pose_to_sim(robot.data.default_root_state[:, :7])
                robot.write_root_velocity_to_sim(torch.zeros(1, 6, device=sim.device))
                robot.write_joint_state_to_sim(
                    robot.data.default_joint_pos,
                    robot.data.default_joint_vel,
                )
                print("[INFO] Reset robot for external control handover", flush=True)
            targets_isaac = external_targets
        elif ort_session is not None and step_count > 0:
            # Internal ONNX policy
            root_lin_vel = robot.data.root_lin_vel_b[0]
            isaac_direct_ang_vel = robot.data.root_ang_vel_b[0]
            isaac_direct_gravity = robot.data.projected_gravity_b[0]
            imu_ang_vel, imu_gravity = build_imu_equivalent_obs_terms(
                robot.data.root_quat_w[0], isaac_direct_ang_vel
            )
            if args_cli.obs_from_imu:
                root_ang_vel, projected_gravity = imu_ang_vel, imu_gravity
            else:
                root_ang_vel = isaac_direct_ang_vel
                projected_gravity = isaac_direct_gravity

            if args_cli.log_imu_obs and (step_count <= 5 or step_count % max(1, args_cli.log_imu_obs_every) == 0):
                used_src = "imu-equivalent" if args_cli.obs_from_imu else "isaac-direct"
                print(
                    f"[OBS {step_count}] src={used_src} "
                    f"lin={root_lin_vel.detach().cpu().numpy().round(4).tolist()} "
                    f"ang_used={root_ang_vel.detach().cpu().numpy().round(4).tolist()} "
                    f"grav_used={projected_gravity.detach().cpu().numpy().round(4).tolist()}",
                    flush=True,
                )
                print(
                    f"[OBS {step_count}] isaac_ang={isaac_direct_ang_vel.detach().cpu().numpy().round(4).tolist()} "
                    f"imu_ang={imu_ang_vel.detach().cpu().numpy().round(4).tolist()} "
                    f"delta_ang={(imu_ang_vel - isaac_direct_ang_vel).detach().cpu().numpy().round(4).tolist()}",
                    flush=True,
                )
                print(
                    f"[OBS {step_count}] isaac_grav={isaac_direct_gravity.detach().cpu().numpy().round(4).tolist()} "
                    f"imu_grav={imu_gravity.detach().cpu().numpy().round(4).tolist()} "
                    f"delta_grav={(imu_gravity - isaac_direct_gravity).detach().cpu().numpy().round(4).tolist()}",
                    flush=True,
                )

            isaac_pos = robot.data.joint_pos[0, :25]
            isaac_vel = robot.data.joint_vel[0, :25]
            hw_pos = isaac_pos[isaac_to_hw]
            hw_vel = isaac_vel[isaac_to_hw]
            joint_pos_rel = hw_pos - default_pos_hw

            obs = torch.cat([
                root_lin_vel, root_ang_vel, projected_gravity,
                commands[0], joint_pos_rel, hw_vel, prev_action[0],
            ]).cpu().numpy().reshape(1, -1).astype(np.float32)

            actions = ort_session.run(None, {"obs": obs})[0]
            actions = np.clip(actions, -5.0, 5.0)
            action_hw = torch.tensor(actions, device=sim.device, dtype=torch.float32).squeeze(0)
            prev_action[0] = action_hw

            targets_hw = default_pos_hw + 0.25 * action_hw
            targets_isaac = targets_hw[hw_to_isaac]
        else:
            # No policy, no external — hold default pose
            targets_isaac = default_targets_isaac

        # --- 3. Apply targets (only if computed) ---
        if targets_isaac is not None:
            robot.set_joint_position_target(
                targets_isaac.unsqueeze(0),
                joint_ids=actuated_joint_ids
            )
        if step_count < 3:
            t = targets_isaac[:6].cpu().numpy().round(3).tolist() if targets_isaac is not None else "None"
            print(f"[step {step_count}] targets={t}, height={robot.data.root_pos_w[0, 2].item():.3f}", flush=True)

        # --- 4. Step simulation ---
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_cfg.dt)
        step_count += 1

        # Send joint_states + IMU via UDP at 200Hz (every physics step)
        if True:
            joint_pos = robot.data.joint_pos[0, :25].cpu().numpy()
            joint_vel = robot.data.joint_vel[0, :25].cpu().numpy()
            quat = robot.data.root_quat_w[0].cpu().numpy()   # w,x,y,z
            ang_vel = robot.data.root_ang_vel_b[0].cpu().numpy()
            lin_vel = robot.data.root_lin_vel_b[0].cpu().numpy()
            # Pack: 25 pos + 25 vel + 4 quat + 3 ang_vel + 3 lin_vel = 60 floats
            data = struct.pack(f'{60}f',
                               *joint_pos, *joint_vel,
                               quat[0], quat[1], quat[2], quat[3],
                               ang_vel[0], ang_vel[1], ang_vel[2],
                               lin_vel[0], lin_vel[1], lin_vel[2])
            try:
                udp_out.sendto(data, out_addr)
                udp_out.sendto(data, out_addr2)
            except OSError:
                pass

        # Periodic log
        if step_count % 500 == 0:
            height = robot.data.root_pos_w[0, 2].item()
            grav = robot.data.projected_gravity_b[0].cpu().numpy()
            print(f"[step {step_count}] height={height:.3f}m, gravity={grav.round(3).tolist()}")



if __name__ == "__main__":
    main()
    simulation_app.close()
