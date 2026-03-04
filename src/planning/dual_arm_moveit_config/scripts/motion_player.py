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
        self._controller_name = args.controller or playback.get(
            "controller_name", "left_arm_group_controller"
        )
        self._planning_group = args.planning_group or playback.get(
            "planning_group", "left_arm"
        )
        self._velocity_scaling = args.velocity_scaling or playback.get(
            "velocity_scaling", 0.3
        )
        self._accel_scaling = args.accel_scaling or playback.get(
            "acceleration_scaling", 0.3
        )
        if args.speed is None:
            self._speed = playback.get("speed_scale", 1.0)

        # Joint state subscriber for preflight check
        self._joint_names = joint_names
        self._joint_index = {name: i for i, name in enumerate(joint_names)}
        self._lock = threading.Lock()
        self._current_positions = None

        self._sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

        # Action clients
        if self._mode == "direct":
            action_topic = f"/{self._controller_name}/follow_joint_trajectory"
            self._action_client = ActionClient(
                self, FollowJointTrajectory, action_topic
            )
        else:
            self._action_client = ActionClient(self, MoveGroup, "move_action")

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

    def _preflight_check(self):
        """Check if arm is near the motion start position."""
        if not self._wait_for_joint_states():
            return False

        first_wp = self._motion["waypoints"][0]["positions"]
        with self._lock:
            current = list(self._current_positions)

        displacements = [abs(c - t) for c, t in zip(current, first_wp)]
        max_disp = max(displacements)

        if max_disp > 0.5:
            self.get_logger().warn(
                f"Arm is {max_disp * 57.2958:.1f} deg from motion start position!"
            )
            self.get_logger().warn(
                "Per-joint deltas (deg): "
                + ", ".join(f"{d * 57.2958:.1f}" for d in displacements)
            )
            print("\n  WARNING: Arm is far from the starting position.")
            print("  The robot will move to the start first.")
            try:
                input("  Press ENTER to continue, Ctrl+C to abort... ")
            except (EOFError, KeyboardInterrupt):
                self.get_logger().info("Playback cancelled.")
                return False

        return True

    def play_direct(self):
        """Playback via FollowJointTrajectory action."""
        waypoints = self._motion["waypoints"]

        # Build JointTrajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = list(self._joint_names)

        for wp in waypoints:
            point = JointTrajectoryPoint()
            point.positions = list(wp["positions"])

            # Apply speed scaling
            t_scaled = wp["t"] / self._speed
            sec = int(t_scaled)
            nsec = int((t_scaled - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nsec)

            trajectory.points.append(point)

        # Set zero velocity at start and end
        if trajectory.points:
            n = len(self._joint_names)
            trajectory.points[0].velocities = [0.0] * n
            trajectory.points[-1].velocities = [0.0] * n

        # Build goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = Duration(sec=5, nanosec=0)

        # Wait for action server
        action_topic = f"/{self._controller_name}/follow_joint_trajectory"
        self.get_logger().info(f"Waiting for action server: {action_topic}...")
        if not self._action_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("Action server not available!")
            return False

        # Send goal
        total_time = waypoints[-1]["t"] / self._speed
        self.get_logger().info(
            f"Sending trajectory: {len(waypoints)} points, "
            f"{total_time:.1f}s (speed: {self._speed}x)"
        )
        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal REJECTED by controller.")
            return False

        self.get_logger().info("Goal accepted, executing...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Playback complete!")
            return True
        else:
            self.get_logger().error(f"Playback failed with status: {result.status}")
            if hasattr(result.result, "error_string"):
                self.get_logger().error(f"Error: {result.result.error_string}")
            return False

    def play_moveit(self):
        """Playback via MoveGroup action, stepping through waypoints."""
        waypoints = self._motion["waypoints"]

        # Downsample: select every Nth waypoint
        step = self._waypoint_step
        if step is None:
            # Auto: aim for 10-15 waypoints
            step = max(1, len(waypoints) // 12)

        selected = list(range(0, len(waypoints), step))
        # Always include last waypoint
        if selected[-1] != len(waypoints) - 1:
            selected.append(len(waypoints) - 1)

        self.get_logger().info(
            f"MoveIt playback: {len(selected)}/{len(waypoints)} waypoints "
            f"(step={step})"
        )

        # Wait for action server
        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self._action_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("MoveGroup action server not available!")
            return False

        succeeded = 0
        failed = 0

        for i, wp_idx in enumerate(selected):
            wp = waypoints[wp_idx]
            self.get_logger().info(
                f"--- Waypoint {i + 1}/{len(selected)} (t={wp['t']:.2f}s) ---"
            )

            # Build joint constraints
            constraints = Constraints()
            for joint_name, value in zip(self._joint_names, wp["positions"]):
                jc = JointConstraint()
                jc.joint_name = joint_name
                jc.position = value
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)

            # Build motion plan request
            req = MotionPlanRequest()
            req.group_name = self._planning_group
            req.num_planning_attempts = 5
            req.allowed_planning_time = 5.0
            req.max_velocity_scaling_factor = self._velocity_scaling
            req.max_acceleration_scaling_factor = self._accel_scaling
            req.goal_constraints.append(constraints)

            # Build MoveGroup goal
            goal = MoveGroup.Goal()
            goal.request = req
            goal.planning_options = PlanningOptions()
            goal.planning_options.plan_only = False
            goal.planning_options.replan = True
            goal.planning_options.replan_attempts = 3

            # Send goal
            send_future = self._action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)

            goal_handle = send_future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f"Waypoint {i + 1} REJECTED.")
                failed += 1
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            result = result_future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                error_code = result.result.error_code.val
                if error_code == 1:  # SUCCESS
                    self.get_logger().info(f"Waypoint {i + 1} reached.")
                    succeeded += 1
                else:
                    self.get_logger().error(
                        f"Waypoint {i + 1} failed (MoveIt error: {error_code})"
                    )
                    failed += 1
            else:
                self.get_logger().error(
                    f"Waypoint {i + 1} ended with status: {result.status}"
                )
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

        if not self._preflight_check():
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
