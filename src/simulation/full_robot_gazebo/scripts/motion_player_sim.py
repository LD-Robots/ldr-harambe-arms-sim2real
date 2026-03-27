#!/usr/bin/env python3
"""
Play back a recorded YAML motion file via whole_body_controller (JointGroupPositionController).

Publishes Float64MultiArray to /whole_body_controller/commands at a configurable rate,
with linear interpolation between waypoints. For joints not in the motion file,
holds the position read from /joint_states at startup.

Usage:
    ros2 run full_robot_gazebo motion_player_sim.py motions/wave.yaml
    ros2 run full_robot_gazebo motion_player_sim.py motions/wave.yaml --speed 0.5 --loop
    ros2 run full_robot_gazebo motion_player_sim.py motions/wave.yaml --dry-run
"""

import argparse
import os
import sys
import threading

import yaml

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# Whole body controller joint order — must match controllers_position.yaml exactly.
WHOLE_BODY_JOINT_ORDER = [
    # Left arm (0-5)
    "left_shoulder_pitch_joint_X6",
    "left_shoulder_roll_joint_X6",
    "left_shoulder_yaw_joint_X4",
    "left_elbow_pitch_joint_X6",
    "left_wrist_yaw_joint_X4",
    "left_wrist_roll_joint_X4",
    # Right arm (6-11)
    "right_shoulder_pitch_joint_X6",
    "right_shoulder_roll_joint_X6",
    "right_shoulder_yaw_joint_X4",
    "right_elbow_pitch_joint_X6",
    "right_wrist_yaw_joint_X4",
    "right_wrist_roll_joint_X4",
    # Waist (12)
    "waist_yaw_joint_X8",
    # Left leg (13-18)
    "left_hip_pitch_joint_X8",
    "left_hip_roll_joint_X8",
    "left_hip_yaw_joint_X8",
    "left_knee_joint_X8",
    "left_ankle_pitch_joint_X4",
    "left_ankle_roll_joint_X4",
    # Right leg (19-24)
    "right_hip_pitch_joint_X8",
    "right_hip_roll_joint_X8",
    "right_hip_yaw_joint_X8",
    "right_knee_joint_X8",
    "right_ankle_pitch_joint_X4",
    "right_ankle_roll_joint_X4",
]

WB_JOINT_TO_IDX = {name: i for i, name in enumerate(WHOLE_BODY_JOINT_ORDER)}
NUM_WB_JOINTS = len(WHOLE_BODY_JOINT_ORDER)


class MotionPlayerSim(Node):

    def __init__(self, args):
        super().__init__(
            "motion_player_sim",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, True),
            ],
        )

        self._speed = args.speed
        self._loop = args.loop
        self._dry_run = args.dry_run
        self._rate = args.rate
        self._go_to_duration = args.go_to_duration
        self._topic = args.topic

        # Load motion file
        self._motion = self._load_motion(args.motion_file)
        if self._motion is None:
            return

        joint_names = self._motion["joint_names"]
        playback = self._motion.get("playback", {})

        # Speed from CLI or file default
        if self._speed is None:
            self._speed = playback.get("speed_scale", 1.0)

        # Build mapping: motion joint index -> whole_body_controller index
        self._motion_to_wb = []  # list of (motion_idx, wb_idx)
        for i, name in enumerate(joint_names):
            if name in WB_JOINT_TO_IDX:
                self._motion_to_wb.append((i, WB_JOINT_TO_IDX[name]))
            else:
                self.get_logger().warn(
                    f"Motion joint '{name}' not in whole_body_controller — skipping"
                )

        if not self._motion_to_wb:
            self.get_logger().error("No motion joints match whole_body_controller joints.")
            self._motion = None
            return

        mapped_names = [joint_names[mi] for mi, _ in self._motion_to_wb]
        self.get_logger().info(
            f"Mapped {len(self._motion_to_wb)}/{len(joint_names)} motion joints "
            f"to whole_body_controller"
        )
        for mi, wi in self._motion_to_wb:
            self.get_logger().info(f"  [{wi:2d}] {joint_names[mi]}")

        # Joint state tracking
        self._lock = threading.Lock()
        self._hold_positions = [0.0] * NUM_WB_JOINTS
        self._joints_received = set()
        self._initial_ready = False

        self._sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

        # Publisher
        self._pub = self.create_publisher(Float64MultiArray, self._topic, 10)

        # Playback state
        self._timer = None
        self._done_event = threading.Event()

    def _load_motion(self, path):
        if not os.path.isfile(path):
            self.get_logger().error(f"Motion file not found: {path}")
            return None

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        motion = data.get("motion")
        if motion is None:
            self.get_logger().error("Invalid motion file: missing 'motion' key")
            return None

        for key in ("joint_names", "waypoints"):
            if key not in motion:
                self.get_logger().error(f"Invalid motion file: missing '{key}'")
                return None

        if not motion["waypoints"]:
            self.get_logger().error("Motion file has no waypoints")
            return None

        self.get_logger().info(
            f"Loaded motion '{motion.get('name', 'unnamed')}': "
            f"{len(motion['waypoints'])} waypoints, "
            f"{motion.get('total_duration_sec', '?')}s"
        )
        return motion

    def _joint_state_cb(self, msg):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in WB_JOINT_TO_IDX:
                    idx = WB_JOINT_TO_IDX[name]
                    self._hold_positions[idx] = float(pos)
                    self._joints_received.add(idx)

            if not self._initial_ready and len(self._joints_received) >= NUM_WB_JOINTS:
                self._initial_ready = True

    def _wait_for_joint_states(self, timeout_sec=15.0):
        self.get_logger().info("Waiting for /joint_states...")
        start = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=timeout_sec)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._lock:
                if self._initial_ready:
                    return True
                received = len(self._joints_received)
            if (self.get_clock().now() - start) > timeout:
                self.get_logger().warn(
                    f"Timeout waiting for all {NUM_WB_JOINTS} joints "
                    f"(got {received}). Proceeding with available data."
                )
                return True  # proceed anyway with whatever we have
        return False

    def _interpolate_waypoint(self, elapsed):
        waypoints = self._motion["waypoints"]
        speed = self._speed

        # Clamp to first/last waypoint
        t_first = waypoints[0]["t"] / speed
        t_last = waypoints[-1]["t"] / speed

        if elapsed <= t_first:
            return waypoints[0]["positions"]
        if elapsed >= t_last:
            return waypoints[-1]["positions"]

        # Find bracketing waypoints
        for i in range(len(waypoints) - 1):
            t0 = waypoints[i]["t"] / speed
            t1 = waypoints[i + 1]["t"] / speed
            if t0 <= elapsed <= t1:
                dt = t1 - t0
                if dt < 1e-9:
                    return waypoints[i]["positions"]
                alpha = (elapsed - t0) / dt
                p0 = waypoints[i]["positions"]
                p1 = waypoints[i + 1]["positions"]
                return [a + (b - a) * alpha for a, b in zip(p0, p1)]

        return waypoints[-1]["positions"]

    def _build_command(self, motion_positions):
        with self._lock:
            cmd = list(self._hold_positions)
        for mi, wi in self._motion_to_wb:
            cmd[wi] = motion_positions[mi]
        return cmd

    def _publish_command(self, positions):
        msg = Float64MultiArray()
        msg.data = positions
        self._pub.publish(msg)

    def _go_to_start(self):
        if not self._wait_for_joint_states():
            return False

        first_wp = self._motion["waypoints"][0]["positions"]
        target_cmd = self._build_command(first_wp)

        with self._lock:
            current = list(self._hold_positions)

        displacements = [abs(c - t) for c, t in zip(current, target_cmd)]
        max_disp = max(displacements)

        if max_disp < 0.02:
            self.get_logger().info("Already at start position.")
            return True

        self.get_logger().info(
            f"Moving to start position (max displacement: "
            f"{max_disp * 57.2958:.1f} deg)..."
        )

        if max_disp > 0.5:
            # Show per-joint deltas for mapped joints only
            motion_joints = self._motion["joint_names"]
            self.get_logger().warn("Per-joint deltas (deg):")
            for mi, wi in self._motion_to_wb:
                d = abs(current[wi] - target_cmd[wi])
                if d > 0.01:
                    self.get_logger().warn(
                        f"  {motion_joints[mi]}: {d * 57.2958:.1f}"
                    )
            try:
                input("  Press ENTER to move to start, Ctrl+C to abort... ")
            except (EOFError, KeyboardInterrupt):
                self.get_logger().info("Playback cancelled.")
                return False

        # Interpolate from current to target over go_to_duration
        start_cmd = list(current)
        duration = self._go_to_duration
        self._done_event.clear()
        start_time = self.get_clock().now()

        def go_to_cb():
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            alpha = min(1.0, elapsed / duration)
            cmd = [s + (t - s) * alpha for s, t in zip(start_cmd, target_cmd)]
            self._publish_command(cmd)
            if alpha >= 1.0:
                self._timer.cancel()
                self._done_event.set()

        self._timer = self.create_timer(1.0 / self._rate, go_to_cb)

        while rclpy.ok() and not self._done_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.05)

        self._timer.cancel()
        self._timer = None
        self.get_logger().info("At start position.")
        return True

    def _play(self):
        waypoints = self._motion["waypoints"]
        total_time = waypoints[-1]["t"] / self._speed

        self.get_logger().info(
            f"Playing: {len(waypoints)} waypoints, "
            f"{total_time:.1f}s (speed: {self._speed}x, rate: {self._rate} Hz)"
        )

        self._done_event.clear()
        start_time = self.get_clock().now()

        def play_cb():
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            motion_pos = self._interpolate_waypoint(elapsed)
            cmd = self._build_command(motion_pos)
            self._publish_command(cmd)

            if elapsed >= total_time:
                self._timer.cancel()
                self._done_event.set()

        self._timer = self.create_timer(1.0 / self._rate, play_cb)

        while rclpy.ok() and not self._done_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.05)

        self._timer.cancel()
        self._timer = None
        return True

    def execute(self):
        if self._motion is None:
            return 1

        if self._dry_run:
            self._print_summary()
            return 0

        if not self._go_to_start():
            return 1

        iteration = 0
        while True:
            iteration += 1
            if self._loop:
                self.get_logger().info(f"=== Loop iteration {iteration} ===")

            ok = self._play()
            if not ok:
                self.get_logger().error("Playback failed.")
                return 1

            if not self._loop:
                break

        self.get_logger().info("Playback complete.")
        return 0

    def _print_summary(self):
        m = self._motion
        waypoints = m["waypoints"]
        print("\n" + "=" * 60)
        print("  MOTION FILE SUMMARY (dry run)")
        print("=" * 60)
        print(f"  Name:        {m.get('name', 'unnamed')}")
        print(f"  Description: {m.get('description', '')}")
        print(f"  Joints:      {len(m['joint_names'])}")
        print(f"  Waypoints:   {len(waypoints)}")
        print(f"  Duration:    {m.get('total_duration_sec', '?')}s")
        print(f"  Sample rate: {m.get('sample_rate_hz', '?')} Hz")
        print(f"\n  Speed scale:     {self._speed}x")
        scaled_dur = waypoints[-1]["t"] / self._speed if waypoints else 0
        print(f"  Scaled duration: {scaled_dur:.1f}s")
        print(f"  Publish rate:    {self._rate} Hz")
        print(f"  Loop:            {self._loop}")
        print(f"  Command topic:   {self._topic}")

        print(f"\n  Joint mapping ({len(self._motion_to_wb)} mapped):")
        for mi, wi in self._motion_to_wb:
            print(f"    [{wi:2d}] {m['joint_names'][mi]}")

        held = set(range(NUM_WB_JOINTS)) - {wi for _, wi in self._motion_to_wb}
        if held:
            print(f"\n  Held joints ({len(held)}):")
            for wi in sorted(held):
                print(f"    [{wi:2d}] {WHOLE_BODY_JOINT_ORDER[wi]}")

        if waypoints:
            print(f"\n  First waypoint: t={waypoints[0]['t']}")
            print(f"  Last waypoint:  t={waypoints[-1]['t']}")
        print("=" * 60 + "\n")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Play back a recorded YAML motion file via whole_body_controller."
    )
    parser.add_argument(
        "motion_file",
        help="Path to YAML motion file",
    )
    parser.add_argument(
        "--speed", type=float, default=None,
        help="Speed scaling factor (default: from file or 1.0)",
    )
    parser.add_argument(
        "--rate", type=int, default=50,
        help="Publish rate in Hz (default: 50)",
    )
    parser.add_argument(
        "--loop", action="store_true",
        help="Loop playback continuously",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Load and validate file without executing",
    )
    parser.add_argument(
        "--go-to-duration", type=float, default=3.0,
        help="Duration in seconds to move to start position (default: 3.0)",
    )
    parser.add_argument(
        "--topic", default="/whole_body_controller/commands",
        help="Command topic (default: /whole_body_controller/commands)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = MotionPlayerSim(args)
    try:
        exit_code = node.execute()
    except KeyboardInterrupt:
        node.get_logger().info("Playback interrupted.")
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
