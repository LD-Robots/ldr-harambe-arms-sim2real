#!/usr/bin/env python3
"""obs_csv_logger — write the SAME obs CSV on the real robot as the sims.

The Gazebo plugin and play_mjx.py / play_isaac_urdf.py --csv all emit one row per
policy step with columns:

    policy_tick, <52 obs fields>, tau.<12 leg joints>

This node reproduces that exact format from the real-robot policy controller's
`~/debug` topic (std_msgs/Float32MultiArray, 100 floats laid out as
[targets(12) | positions(12) | velocities(12) | last_action(12) | obs(52)]), so a
real run drops straight into scripts/plot_csv_gui.py next to the sim CSVs.

`tau` (applied joint torque) is not on the wire — the controller is position-PD
and the drive computes torque — so we reconstruct the SAME quantity the sims log:
    tau[i] = clip(kp[i]*(target[i]-pos[i]) - kd[i]*vel[i], -eff[i], +eff[i])
using the per-joint kp/kd/effort from the deploy contract (override via params to
match a non-default controller config).

Run (csv_path is optional — defaults to /tmp/csv_real/obs_real_<timestamp>.csv):
    ros2 run harambe_policy_legs_controller obs_csv_logger.py
    ros2 run harambe_policy_legs_controller obs_csv_logger.py \
        --ros-args -p csv_path:=/tmp/csv_real/myrun.csv

The controller must have publish_debug:=true. robot_pvt_policy.launch.py logs by
default (log_csv:=true); pass log_csv:=false to turn it off.
"""
import csv
import datetime
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

DEFAULT_DIR = "/tmp/csv_real"

# Leg joints in training order — MUST match the controller `joints:` and the sim
# _MJX_LEG_JOINTS so the columns line up for a sim-vs-real diff.
LEG_JOINTS = [
    "left_hip_pitch_joint_X8", "left_hip_roll_joint_X8", "left_hip_yaw_joint_X8",
    "left_knee_joint_X8", "left_ankle_pitch_joint_X4", "left_ankle_roll_joint_X4",
    "right_hip_pitch_joint_X8", "right_hip_roll_joint_X8", "right_hip_yaw_joint_X8",
    "right_knee_joint_X8", "right_ankle_pitch_joint_X4", "right_ankle_roll_joint_X4",
]
NJ = 12
OBS_DIM = 52

# Deploy-contract defaults (match harambe_policy_legs_controller.yaml).
DEFAULT_KP = [75.0, 75.0, 75.0, 75.0, 20.0, 20.0,
              75.0, 75.0, 75.0, 75.0, 20.0, 20.0]
DEFAULT_KD = [5.625, 6.162, 3.750, 5.511, 1.414, 1.000,
              5.625, 6.162, 3.750, 5.511, 1.414, 1.000]
DEFAULT_EFF = [120.0, 120.0, 120.0, 120.0, 80.0, 80.0,
               120.0, 120.0, 120.0, 120.0, 80.0, 80.0]


def csv_obs_header():
    """Identical to play_isaac_urdf.csv_obs_header(52) / the Gazebo plugin."""
    cols = ["policy_tick",
            "base_lin_vel.x", "base_lin_vel.y", "base_lin_vel.z",
            "base_ang_vel.x", "base_ang_vel.y", "base_ang_vel.z",
            "projected_grav.x", "projected_grav.y", "projected_grav.z",
            "cmd.vx", "cmd.vy", "cmd.yaw"]
    for sec in ("qpos_rel", "qvel", "last_action"):
        cols += [f"{sec}.{n}" for n in LEG_JOINTS]
    cols += ["gait.cos_phL", "gait.cos_phR", "gait.sin_phL", "gait.sin_phR"]
    cols += [f"tau.{n}" for n in LEG_JOINTS]
    return cols


class ObsCsvLogger(Node):
    def __init__(self):
        super().__init__("obs_csv_logger")
        # csv_path: "" → auto /tmp/csv_real/obs_real_<timestamp>.csv ; a directory
        # → <dir>/obs_real_<timestamp>.csv ; a *.csv file → used verbatim.
        self.declare_parameter("csv_path", "")
        self.declare_parameter("debug_topic",
                               "/harambe_policy_legs_controller/debug")
        self.declare_parameter("kp", DEFAULT_KP)
        self.declare_parameter("kd", DEFAULT_KD)
        self.declare_parameter("effort_limits", DEFAULT_EFF)

        self.kp = list(self.get_parameter("kp").value)
        self.kd = list(self.get_parameter("kd").value)
        self.eff = list(self.get_parameter("effort_limits").value)
        path = self._resolve_path(self.get_parameter("csv_path").value)
        topic = self.get_parameter("debug_topic").value

        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.fh = open(path, "w", newline="")
        self.writer = csv.writer(self.fh)
        self.writer.writerow(csv_obs_header())
        self.fh.flush()
        self.tick = 0

        self.sub = self.create_subscription(
            Float32MultiArray, topic, self._on_debug, 50)
        self.get_logger().info(
            f"obs_csv_logger: '{topic}' -> {path} "
            f"(same columns as the sims; tau reconstructed from kp/kd)")

    @staticmethod
    def _resolve_path(path):
        """Empty → /tmp/csv_real/obs_real_<ts>.csv ; directory → <dir>/...<ts>.csv ;
        explicit *.csv → verbatim. Timestamp makes every run a unique file."""
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = f"obs_real_{ts}.csv"
        if not path:
            return os.path.join(DEFAULT_DIR, fname)
        if path.endswith(os.sep) or os.path.isdir(path):
            return os.path.join(path, fname)
        return path

    def _on_debug(self, msg):
        d = msg.data
        if len(d) != 4 * NJ + OBS_DIM:        # 100
            self.get_logger().warn(
                f"debug msg has {len(d)} floats, expected {4 * NJ + OBS_DIM}; "
                "is this the policy controller's ~/debug?", once=True)
            return
        targets = d[0:NJ]
        positions = d[NJ:2 * NJ]
        velocities = d[2 * NJ:3 * NJ]
        obs = d[4 * NJ:4 * NJ + OBS_DIM]      # the 52-dim observation

        tau = []
        for i in range(NJ):
            t = self.kp[i] * (targets[i] - positions[i]) - self.kd[i] * velocities[i]
            t = max(-self.eff[i], min(self.eff[i], t))
            tau.append(t)

        self.writer.writerow([self.tick] + list(obs) + tau)
        self.fh.flush()
        self.tick += 1

    def destroy_node(self):
        try:
            self.fh.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = ObsCsvLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
