#!/usr/bin/env python3
"""
Play back a recorded YAML motion file via direct controller or MoveIt.

Supports speed scaling, looping, and two playback modes:
  - direct: Sends JointTrajectory to FollowJointTrajectory action (fast)
  - moveit: Steps through waypoints via MoveGroup action (collision-aware)

Usage:
    ros2 run dual_arm_moveit_config motion_player.py motions/wave.yaml
    ros2 run dual_arm_moveit_config motion_player.py motions/wave.yaml --mode moveit
    ros2 run dual_arm_moveit_config motion_player.py motions/wave.yaml --speed 0.5 --loop
"""

import argparse
import math
import os
import sys
import threading

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



# Known body groups: each maps joint names to a controller and MoveIt planning group.
# Joints are matched by name from the motion file — groups with no matching joints
# are skipped automatically.
BODY_GROUPS = {
    "left_arm": {
        "controller": "left_arm_group_controller",
        "planning_group": "left_arm",
        "joint_names": [
            "left_shoulder_pitch_joint_X6",
            "left_shoulder_roll_joint_X6",
            "left_shoulder_yaw_joint_X4",
            "left_elbow_pitch_joint_X6",
            "left_wrist_yaw_joint_X4",
            "left_wrist_roll_joint_X4",
        ],
    },
    "right_arm": {
        "controller": "right_arm_group_controller",
        "planning_group": "right_arm",
        "joint_names": [
            "right_shoulder_pitch_joint_X6",
            "right_shoulder_roll_joint_X6",
            "right_shoulder_yaw_joint_X4",
            "right_elbow_pitch_joint_X6",
            "right_wrist_yaw_joint_X4",
            "right_wrist_roll_joint_X4",
        ],
    },
    "waist": {
        "controller": "waist_controller",
        "planning_group": "waist",
        "joint_names": [
            "waist_yaw_joint_X8",
        ],
    },
    "left_leg": {
        "controller": "left_leg_group_controller",
        "planning_group": "left_leg",
        "joint_names": [
            "left_hip_pitch_joint_X8",
            "left_hip_roll_joint_X8",
            "left_hip_yaw_joint_X8",
            "left_knee_joint_X8",
            "left_ankle_pitch_joint_X4",
            "left_ankle_roll_joint_X4",
        ],
    },
    "right_leg": {
        "controller": "right_leg_group_controller",
        "planning_group": "right_leg",
        "joint_names": [
            "right_hip_pitch_joint_X8",
            "right_hip_roll_joint_X8",
            "right_hip_yaw_joint_X8",
            "right_knee_joint_X8",
            "right_ankle_roll_joint_X4",
            # right_ankle_pitch_joint_X4 — not connected yet (bus 24)
        ],
    },
}


def _detect_groups(joint_names):
    """Detect which body groups are present in the motion file's joint list.

    Returns a dict of group_key -> {controller, planning_group, joint_names, indices}
    where indices are positions in the motion file's joint_names list.
    """
    joint_set = set(joint_names)
    joint_to_idx = {name: i for i, name in enumerate(joint_names)}
    groups = {}

    for key, group_def in BODY_GROUPS.items():
        matched = [j for j in group_def["joint_names"] if j in joint_set]
        if matched:
            groups[key] = {
                "controller": group_def["controller"],
                "planning_group": group_def["planning_group"],
                "joint_names": matched,
                "indices": [joint_to_idx[j] for j in matched],
            }

    return groups


class MotionPlayer(Node):

    def __init__(self, args):
        super().__init__("motion_player")

        self._mode = args.mode
        self._speed = args.speed
        self._loop = args.loop
        self._dry_run = args.dry_run
        self._waypoint_step = args.waypoint_step

        # Load motion file
        self._motion = self._load_motion(args.motion_file)
        if self._motion is None:
            return

        joint_names = self._motion["joint_names"]
        playback = self._motion.get("playback", {})

        # Override from CLI or use file defaults
        self._velocity_scaling = args.velocity_scaling or playback.get(
            "velocity_scaling", 0.3
        )
        self._accel_scaling = args.accel_scaling or playback.get(
            "acceleration_scaling", 0.3
        )
        if args.speed is None:
            self._speed = playback.get("speed_scale", 1.0)

        # Detect body groups from joint names in the motion file
        if args.controller:
            # CLI override: treat all joints as a single group
            self._arm_groups = {
                "single": {
                    "controller": args.controller,
                    "planning_group": args.planning_group or "left_arm",
                    "joint_names": list(joint_names),
                    "indices": list(range(len(joint_names))),
                },
            }
        else:
            self._arm_groups = _detect_groups(joint_names)
            if not self._arm_groups:
                self.get_logger().error(
                    "No known body groups found in motion file joint names. "
                    "Use --controller to specify a controller manually."
                )
                self._motion = None
                return

            for key, group in self._arm_groups.items():
                self.get_logger().info(
                    f"  {key}: {len(group['joint_names'])} joints "
                    f"-> {group['controller']}"
                )

        # Joint state subscriber for preflight check
        self._joint_names = joint_names
        self._joint_index = {name: i for i, name in enumerate(joint_names)}
        self._lock = threading.Lock()
        self._current_positions = None

        self._sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

        # Action clients — one per arm group
        self._action_clients = {}
        if self._mode == "direct":
            for key, group in self._arm_groups.items():
                topic = f"/{group['controller']}/follow_joint_trajectory"
                self._action_clients[key] = ActionClient(
                    self, FollowJointTrajectory, topic
                )
        else:
            # MoveIt uses a single action server
            self._action_clients["moveit"] = ActionClient(
                self, MoveGroup, "move_action"
            )

    def _load_motion(self, path):
        """Load and validate YAML motion file."""
        if not os.path.isfile(path):
            self.get_logger().error(f"Motion file not found: {path}")
            return None

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        motion = data.get("motion")
        if motion is None:
            self.get_logger().error("Invalid motion file: missing 'motion' key")
            return None

        required = ["joint_names", "waypoints"]
        for key in required:
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
        """Capture current joint positions for preflight check."""
        if self._current_positions is not None:
            return  # Only need first reading

        positions = [None] * len(self._joint_names)
        for name, pos in zip(msg.name, msg.position):
            if name in self._joint_index:
                positions[self._joint_index[name]] = pos

        if any(v is None for v in positions):
            return

        with self._lock:
            self._current_positions = [float(p) for p in positions]

    def _wait_for_joint_states(self, timeout_sec=10.0):
        """Wait for current joint positions."""
        self.get_logger().info("Waiting for /joint_states...")
        start = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=timeout_sec)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._lock:
                if self._current_positions is not None:
                    return True
            if (self.get_clock().now() - start) > timeout:
                self.get_logger().error(
                    f"Timed out waiting for /joint_states ({timeout_sec}s)"
                )
                return False
        return False

    def _go_to_start(self):
        """Move arms to the first waypoint before playback."""
        if not self._wait_for_joint_states():
            return False

        first_wp = self._motion["waypoints"][0]["positions"]
        with self._lock:
            current = list(self._current_positions)

        displacements = [abs(c - t) for c, t in zip(current, first_wp)]
        max_disp = max(displacements)

        if max_disp < 0.02:
            self.get_logger().info("Already at start position.")
            return True

        self.get_logger().info(
            f"Moving to start position (max displacement: "
            f"{max_disp * 57.2958:.1f} deg)..."
        )

        if max_disp > 0.5:
            self.get_logger().warn(
                "Per-joint deltas (deg): "
                + ", ".join(f"{d * 57.2958:.1f}" for d in displacements)
            )
            try:
                input("  Press ENTER to move to start, Ctrl+C to abort... ")
            except (EOFError, KeyboardInterrupt):
                self.get_logger().info("Playback cancelled.")
                return False

        # Wait for action servers
        for key, client in self._action_clients.items():
            group = self._arm_groups[key]
            topic = f"/{group['controller']}/follow_joint_trajectory"
            if not client.wait_for_server(timeout_sec=30.0):
                self.get_logger().error(f"Action server not available: {topic}")
                return False

        # Compute duration from max displacement: slow and safe (0.2 rad/s)
        move_duration = max(3.0, max_disp / 0.2)

        # Send go-to-start goals to all arm groups in parallel
        send_futures = {}
        for key, client in self._action_clients.items():
            group = self._arm_groups[key]
            indices = group["indices"]

            trajectory = JointTrajectory()
            trajectory.joint_names = list(group["joint_names"])

            # Current position as start
            start_point = JointTrajectoryPoint()
            start_point.positions = [current[i] for i in indices]
            start_point.velocities = [0.0] * len(indices)
            start_point.time_from_start = Duration(sec=0, nanosec=0)
            trajectory.points.append(start_point)

            # First waypoint as target
            end_point = JointTrajectoryPoint()
            end_point.positions = [first_wp[i] for i in indices]
            end_point.velocities = [0.0] * len(indices)
            sec = int(move_duration)
            nsec = int((move_duration - sec) * 1e9)
            end_point.time_from_start = Duration(sec=sec, nanosec=nsec)
            trajectory.points.append(end_point)

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            goal.goal_time_tolerance = Duration(sec=5, nanosec=0)

            send_futures[key] = client.send_goal_async(goal)

        # Wait for acceptance and completion
        for key, future in send_futures.items():
            rclpy.spin_until_future_complete(self, future)
            handle = future.result()
            if not handle.accepted:
                self.get_logger().error(f"Go-to-start REJECTED by {key} controller.")
                return False

            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error(f"Go-to-start failed for {key} arm.")
                return False

        self.get_logger().info("At start position.")
        return True

    def _build_trajectory(self, group, waypoints):
        """Build a JointTrajectory for a single arm group."""
        trajectory = JointTrajectory()
        trajectory.joint_names = list(group["joint_names"])
        indices = group["indices"]

        for wp in waypoints:
            point = JointTrajectoryPoint()
            point.positions = [wp["positions"][i] for i in indices]

            t_scaled = wp["t"] / self._speed
            sec = int(t_scaled)
            nsec = int((t_scaled - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nsec)

            trajectory.points.append(point)

        # Set zero velocity at start and end
        if trajectory.points:
            n = len(group["joint_names"])
            trajectory.points[0].velocities = [0.0] * n
            trajectory.points[-1].velocities = [0.0] * n

        return trajectory

    def play_direct(self):
        """Playback via FollowJointTrajectory action (parallel for dual-arm)."""
        waypoints = self._motion["waypoints"]

        # Wait for all action servers
        for key, client in self._action_clients.items():
            group = self._arm_groups[key]
            topic = f"/{group['controller']}/follow_joint_trajectory"
            self.get_logger().info(f"Waiting for action server: {topic}...")
            if not client.wait_for_server(timeout_sec=30.0):
                self.get_logger().error(f"Action server not available: {topic}")
                return False

        total_time = waypoints[-1]["t"] / self._speed
        self.get_logger().info(
            f"Sending trajectory: {len(waypoints)} points, "
            f"{total_time:.1f}s (speed: {self._speed}x)"
        )

        # Send goals to all arm groups in parallel
        send_futures = {}
        for key, client in self._action_clients.items():
            group = self._arm_groups[key]
            trajectory = self._build_trajectory(group, waypoints)

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            goal.goal_time_tolerance = Duration(sec=5, nanosec=0)

            send_futures[key] = client.send_goal_async(goal)

        # Wait for all goals to be accepted
        goal_handles = {}
        for key, future in send_futures.items():
            rclpy.spin_until_future_complete(self, future)
            handle = future.result()
            if not handle.accepted:
                self.get_logger().error(f"Goal REJECTED by {key} controller.")
                return False
            goal_handles[key] = handle

        self.get_logger().info("All goals accepted, executing...")

        # Wait for all results
        result_futures = {
            key: handle.get_result_async()
            for key, handle in goal_handles.items()
        }

        all_ok = True
        for key, future in result_futures.items():
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"{key} arm playback complete!")
            else:
                self.get_logger().error(
                    f"{key} arm playback failed with status: {result.status}"
                )
                if hasattr(result.result, "error_string"):
                    self.get_logger().error(f"Error: {result.result.error_string}")
                all_ok = False

        return all_ok

    def play_moveit(self):
        """Playback via MoveGroup action, stepping through waypoints."""
        waypoints = self._motion["waypoints"]
        moveit_client = self._action_clients["moveit"]

        # Downsample: select every Nth waypoint
        step = self._waypoint_step
        if step is None:
            step = max(1, len(waypoints) // 12)

        selected = list(range(0, len(waypoints), step))
        if selected[-1] != len(waypoints) - 1:
            selected.append(len(waypoints) - 1)

        self.get_logger().info(
            f"MoveIt playback: {len(selected)}/{len(waypoints)} waypoints "
            f"(step={step})"
        )

        self.get_logger().info("Waiting for MoveGroup action server...")
        if not moveit_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("MoveGroup action server not available!")
            return False

        succeeded = 0
        failed = 0

        for i, wp_idx in enumerate(selected):
            wp = waypoints[wp_idx]
            self.get_logger().info(
                f"--- Waypoint {i + 1}/{len(selected)} (t={wp['t']:.2f}s) ---"
            )

            # Send one MoveGroup goal per arm group
            wp_ok = True
            for key, group in self._arm_groups.items():
                constraints = Constraints()
                for joint_name, idx in zip(group["joint_names"], group["indices"]):
                    jc = JointConstraint()
                    jc.joint_name = joint_name
                    jc.position = wp["positions"][idx]
                    jc.tolerance_above = 0.01
                    jc.tolerance_below = 0.01
                    jc.weight = 1.0
                    constraints.joint_constraints.append(jc)

                req = MotionPlanRequest()
                req.group_name = group["planning_group"]
                req.num_planning_attempts = 5
                req.allowed_planning_time = 5.0
                req.max_velocity_scaling_factor = self._velocity_scaling
                req.max_acceleration_scaling_factor = self._accel_scaling
                req.goal_constraints.append(constraints)

                goal = MoveGroup.Goal()
                goal.request = req
                goal.planning_options = PlanningOptions()
                goal.planning_options.plan_only = False
                goal.planning_options.replan = True
                goal.planning_options.replan_attempts = 3

                send_future = moveit_client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self, send_future)

                goal_handle = send_future.result()
                if not goal_handle.accepted:
                    self.get_logger().error(
                        f"Waypoint {i + 1} ({key}) REJECTED."
                    )
                    wp_ok = False
                    continue

                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)

                result = result_future.result()
                if result.status == GoalStatus.STATUS_SUCCEEDED:
                    error_code = result.result.error_code.val
                    if error_code == 1:
                        self.get_logger().info(
                            f"Waypoint {i + 1} ({key}) reached."
                        )
                    else:
                        self.get_logger().error(
                            f"Waypoint {i + 1} ({key}) failed "
                            f"(MoveIt error: {error_code})"
                        )
                        wp_ok = False
                else:
                    self.get_logger().error(
                        f"Waypoint {i + 1} ({key}) ended with "
                        f"status: {result.status}"
                    )
                    wp_ok = False

            if wp_ok:
                succeeded += 1
            else:
                failed += 1

        self.get_logger().info(
            f"MoveIt playback done: {succeeded}/{succeeded + failed} waypoints reached."
        )
        return failed == 0

    def execute(self):
        """Main entry point."""
        if self._motion is None:
            return 1

        if self._dry_run:
            self._print_summary()
            return 0

        if not self._go_to_start():
            return 1

        play_fn = self.play_direct if self._mode == "direct" else self.play_moveit

        iteration = 0
        while True:
            iteration += 1
            if self._loop:
                self.get_logger().info(f"=== Loop iteration {iteration} ===")

            # Reset current_positions to allow preflight on subsequent loops
            if iteration > 1:
                with self._lock:
                    self._current_positions = None

            ok = play_fn()
            if not ok:
                self.get_logger().error("Playback failed.")
                return 1

            if not self._loop:
                break

        return 0

    def _print_summary(self):
        """Print motion file summary (dry run)."""
        m = self._motion
        waypoints = m["waypoints"]
        print("\n" + "=" * 60)
        print("  MOTION FILE SUMMARY (dry run)")
        print("=" * 60)
        print(f"  Name:       {m.get('name', 'unnamed')}")
        print(f"  Description:{m.get('description', '')}")
        print(f"  Joints:     {len(m['joint_names'])}")
        print(f"  Waypoints:  {len(waypoints)}")
        print(f"  Duration:   {m.get('total_duration_sec', '?')}s")
        print(f"  Sample rate:{m.get('sample_rate_hz', '?')} Hz")
        print(f"\n  Playback mode:  {self._mode}")
        print(f"  Speed scale:    {self._speed}x")
        scaled_dur = waypoints[-1]["t"] / self._speed if waypoints else 0
        print(f"  Scaled duration:{scaled_dur:.1f}s")
        print(f"  Loop:           {self._loop}")

        if waypoints:
            print(f"\n  First waypoint: t={waypoints[0]['t']}")
            print(f"    positions: {waypoints[0]['positions']}")
            print(f"  Last waypoint:  t={waypoints[-1]['t']}")
            print(f"    positions: {waypoints[-1]['positions']}")
        print("=" * 60 + "\n")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Play back a recorded YAML motion file."
    )
    parser.add_argument(
        "motion_file",
        help="Path to YAML motion file"
    )
    parser.add_argument(
        "--mode", choices=["direct", "moveit"], default="direct",
        help="Playback mode (default: direct)"
    )
    parser.add_argument(
        "--speed", type=float, default=None,
        help="Speed scaling factor (default: from file or 1.0)"
    )
    parser.add_argument(
        "--velocity-scaling", type=float, default=None,
        help="MoveIt velocity scaling (default: from file or 0.3)"
    )
    parser.add_argument(
        "--accel-scaling", type=float, default=None,
        help="MoveIt acceleration scaling (default: from file or 0.3)"
    )
    parser.add_argument(
        "--controller", default=None,
        help="Controller name override"
    )
    parser.add_argument(
        "--planning-group", default=None,
        help="MoveIt planning group override"
    )
    parser.add_argument(
        "--loop", action="store_true",
        help="Loop playback continuously"
    )
    parser.add_argument(
        "--waypoint-step", type=int, default=None,
        help="In MoveIt mode, use every Nth waypoint (default: auto)"
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Load and validate file without executing"
    )

    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = MotionPlayer(args)
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
