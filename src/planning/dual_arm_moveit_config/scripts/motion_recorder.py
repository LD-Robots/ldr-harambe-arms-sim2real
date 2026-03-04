#!/usr/bin/env python3
"""
Record joint states to a YAML motion file for later playback.

Subscribes to /joint_states, samples at a configurable rate with dead-band
filtering, and saves timestamped waypoints to YAML on stop.

Works with any /joint_states source: real hardware (position_viewer or
gravity comp mode), simulation (Gazebo), or teleop.

Usage:
    ros2 run dual_arm_moveit_config motion_recorder.py -n wave_hello
    ros2 run dual_arm_moveit_config motion_recorder.py -n pick --rate 20 -o ~/motions/pick.yaml
"""

import argparse
import os
import sys
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Try yaml with C loader for speed
try:
    import yaml
    from yaml import CDumper as Dumper
except ImportError:
    import yaml
    from yaml import Dumper

DEFAULT_JOINTS = [
    "left_shoulder_pitch_joint_X6",
    "left_shoulder_roll_joint_X6",
    "left_shoulder_yaw_joint_X4",
    "left_elbow_pitch_joint_X6",
    "left_wrist_yaw_joint_X4",
    "left_wrist_roll_joint_X4",
]


class CompactListDumper(Dumper):
    """YAML dumper that renders lists of floats in flow (inline) style."""
    pass


def _represent_list(dumper, data):
    """Use flow style for short lists of numbers (waypoint positions)."""
    if all(isinstance(x, (int, float)) for x in data) and len(data) <= 12:
        return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=True)
    return dumper.represent_sequence("tag:yaml.org,2002:seq", data)


CompactListDumper.add_representer(list, _represent_list)


class MotionRecorder(Node):

    def __init__(self, args):
        super().__init__("motion_recorder")

        self._joint_names = args.joints
        self._sample_rate = args.rate
        self._min_change = args.min_change
        self._output_path = args.output
        self._motion_name = args.name
        self._description = args.description
        self._topic = args.topic

        # Joint name → index mapping for fast lookup
        self._joint_index = {name: i for i, name in enumerate(self._joint_names)}
        self._num_joints = len(self._joint_names)

        # Thread-safe state
        self._lock = threading.Lock()
        self._latest_positions = None

        # Recording state
        self._waypoints = []
        self._recording = False
        self._start_time = None
        self._last_recorded = None
        self._stop_event = threading.Event()

        # Subscriber
        self._sub = self.create_subscription(
            JointState, self._topic, self._joint_state_cb, 10
        )

        # Sampling timer (created when recording starts)
        self._timer = None

    def _joint_state_cb(self, msg):
        """Extract target joint positions from /joint_states."""
        positions = [None] * self._num_joints
        for name, pos in zip(msg.name, msg.position):
            if name in self._joint_index:
                positions[self._joint_index[name]] = pos

        if any(v is None for v in positions):
            return

        with self._lock:
            self._latest_positions = [float(p) for p in positions]

    def _sample_timer_cb(self):
        """Timer callback: sample current position and append waypoint if changed."""
        with self._lock:
            if self._latest_positions is None:
                return
            pos = list(self._latest_positions)

        # Dead-band filter: skip if no joint moved significantly
        if self._last_recorded is not None:
            max_delta = max(abs(a - b) for a, b in zip(pos, self._last_recorded))
            if max_delta < self._min_change:
                return

        t = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        self._waypoints.append({
            "t": round(t, 4),
            "positions": [round(p, 6) for p in pos],
        })
        self._last_recorded = pos

        if len(self._waypoints) % 50 == 0:
            self.get_logger().info(f"  {len(self._waypoints)} waypoints recorded...")

    def wait_for_joint_states(self, timeout_sec=10.0):
        """Block until first valid /joint_states message arrives."""
        self.get_logger().info(f"Waiting for {self._topic}...")
        start = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=timeout_sec)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._lock:
                if self._latest_positions is not None:
                    self.get_logger().info("Joint states received.")
                    return True
            if (self.get_clock().now() - start) > timeout:
                self.get_logger().error(
                    f"Timed out waiting for {self._topic} ({timeout_sec}s)"
                )
                return False
        return False

    def start_recording(self):
        """Begin sampling joint positions."""
        with self._lock:
            pos = list(self._latest_positions)

        self._start_time = self.get_clock().now()
        self._waypoints = [{"t": 0.0, "positions": [round(p, 6) for p in pos]}]
        self._last_recorded = pos
        self._recording = True

        period = 1.0 / self._sample_rate
        self._timer = self.create_timer(period, self._sample_timer_cb)

        self.get_logger().info(
            f"Recording started at {self._sample_rate} Hz "
            f"(min change: {self._min_change} rad)"
        )

    def stop_recording(self):
        """Stop sampling and capture final position."""
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        self._recording = False

        # Capture final position
        with self._lock:
            if self._latest_positions is not None:
                pos = list(self._latest_positions)
                t = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
                self._waypoints.append({
                    "t": round(t, 4),
                    "positions": [round(p, 6) for p in pos],
                })

        self.get_logger().info(f"Recording stopped. {len(self._waypoints)} waypoints.")

    def save_yaml(self):
        """Save recorded motion to YAML file."""
        if not self._waypoints:
            self.get_logger().error("No waypoints recorded!")
            return False

        total_duration = self._waypoints[-1]["t"]

        motion = {
            "motion": {
                "name": self._motion_name,
                "description": self._description or "",
                "recorded_at": datetime.now().isoformat(timespec="seconds"),
                "joint_names": list(self._joint_names),
                "sample_rate_hz": self._sample_rate,
                "total_duration_sec": round(total_duration, 4),
                "num_waypoints": len(self._waypoints),
                "playback": {
                    "speed_scale": 1.0,
                    "velocity_scaling": 0.3,
                    "acceleration_scaling": 0.3,
                    "controller_name": "left_arm_group_controller",
                    "planning_group": "left_arm",
                },
                "waypoints": self._waypoints,
            }
        }

        # Ensure output directory exists
        output_dir = os.path.dirname(self._output_path)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        with open(self._output_path, "w") as f:
            yaml.dump(motion, f, Dumper=CompactListDumper, default_flow_style=False, sort_keys=False)

        self.get_logger().info(f"Saved {len(self._waypoints)} waypoints to: {self._output_path}")
        self.get_logger().info(f"Duration: {total_duration:.2f}s")
        return True

    def run(self):
        """Main recording loop with keyboard input."""
        if not self.wait_for_joint_states():
            return 1

        print("\n" + "=" * 60)
        print("  MOTION RECORDER")
        print(f"  Motion name: {self._motion_name}")
        print(f"  Joints: {len(self._joint_names)}")
        print(f"  Sample rate: {self._sample_rate} Hz")
        print(f"  Output: {self._output_path}")
        print("=" * 60)
        print("\n  Press ENTER to START recording...")
        print("  Press ENTER again to STOP and save.")
        print("  Press Ctrl+C to cancel without saving.\n")

        # Wait for ENTER to start
        try:
            input()
        except EOFError:
            return 1

        self.start_recording()

        # Spin in background while waiting for stop signal
        spin_thread = threading.Thread(
            target=self._spin_until_stop, daemon=True
        )
        spin_thread.start()

        # Wait for ENTER to stop
        try:
            input("  >>> Recording... Press ENTER to stop. <<<\n")
        except (EOFError, KeyboardInterrupt):
            pass

        self.stop_recording()
        self._stop_event.set()
        spin_thread.join(timeout=2.0)

        if self.save_yaml():
            return 0
        return 1

    def _spin_until_stop(self):
        """Spin the node until stop event is set."""
        while not self._stop_event.is_set() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record joint states to a YAML motion file."
    )
    parser.add_argument(
        "-n", "--name", required=True,
        help="Motion name (used as filename if --output not given)"
    )
    parser.add_argument(
        "-o", "--output",
        help="Output YAML file path (default: motions/<name>.yaml)"
    )
    parser.add_argument(
        "--rate", type=float, default=10.0,
        help="Recording sample rate in Hz (default: 10)"
    )
    parser.add_argument(
        "--min-change", type=float, default=0.001,
        help="Min joint movement (rad) to record a waypoint (default: 0.001)"
    )
    parser.add_argument(
        "--topic", default="/joint_states",
        help="Joint state topic (default: /joint_states)"
    )
    parser.add_argument(
        "--joints", nargs="+", default=DEFAULT_JOINTS,
        help="Joint names to record"
    )
    parser.add_argument(
        "-d", "--description", default="",
        help="Optional description for the motion"
    )

    args = parser.parse_args()

    # Default output path
    if args.output is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        motions_dir = os.path.join(script_dir, "..", "motions")
        # Fall back to current directory if motions dir doesn't exist
        if not os.path.isdir(motions_dir):
            motions_dir = os.path.join(os.getcwd(), "motions")
        args.output = os.path.join(motions_dir, f"{args.name}.yaml")

    return args


def main():
    args = parse_args()
    rclpy.init()
    node = MotionRecorder(args)
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Recording cancelled.")
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
