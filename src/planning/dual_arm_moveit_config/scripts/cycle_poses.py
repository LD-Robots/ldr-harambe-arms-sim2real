#!/usr/bin/env python3
"""
Cycle through all named poses for the left_arm group using MoveIt.

Parses the SRDF for named group_states, then sends MoveGroup action goals
to plan and execute to each one sequentially.

Requires MoveIt move_group to be running (e.g. via demo.launch.py).

Usage:
    ros2 run dual_arm_moveit_config cycle_poses.py
"""

import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
)


def parse_named_states(srdf_path: str, group: str) -> dict:
    """Parse SRDF and return {state_name: {joint_name: value}} for a group."""
    tree = ET.parse(srdf_path)
    root = tree.getroot()
    states = {}
    for gs in root.findall("group_state"):
        if gs.attrib.get("group") == group:
            name = gs.attrib["name"]
            joints = {}
            for j in gs.findall("joint"):
                joints[j.attrib["name"]] = float(j.attrib["value"])
            states[name] = joints
    return states


class CyclePoses(Node):
    def __init__(self):
        super().__init__("cycle_poses")
        self._action_client = ActionClient(self, MoveGroup, "move_action")

    def move_to_named_state(
        self, state_name: str, joints: dict, planning_time: float = 5.0
    ) -> bool:
        """Plan and execute to a named state. Returns True on success."""
        self.get_logger().info(f"--- Moving to '{state_name}' ---")

        # Build joint constraints from parsed SRDF values
        constraints = Constraints()
        for joint_name, value in joints.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        # Build motion plan request
        req = MotionPlanRequest()
        req.group_name = "left_arm"
        req.num_planning_attempts = 5
        req.allowed_planning_time = planning_time
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.goal_constraints.append(constraints)

        # Build MoveGroup goal
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False  # Plan and execute
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server not available!")
            return False

        # Send goal
        self.get_logger().info(f"Sending goal for '{state_name}'...")
        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal REJECTED for '{state_name}'.")
            return False

        # Wait for result
        self.get_logger().info(f"Goal accepted, executing '{state_name}'...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            error_code = result.result.error_code.val
            if error_code == 1:  # SUCCESS
                self.get_logger().info(f"'{state_name}' reached!")
                return True
            else:
                self.get_logger().error(
                    f"'{state_name}' failed with MoveIt error code: {error_code}"
                )
                return False
        else:
            self.get_logger().error(
                f"'{state_name}' action ended with status: {result.status}"
            )
            return False


def main():
    rclpy.init()
    node = CyclePoses()

    # Parse named states from SRDF
    pkg_share = get_package_share_directory("dual_arm_moveit_config")
    srdf_path = f"{pkg_share}/config/dual_arm_description.srdf"
    named_states = parse_named_states(srdf_path, "left_arm")

    if not named_states:
        node.get_logger().error("No named states found for 'left_arm' in SRDF!")
        node.destroy_node()
        rclpy.shutdown()
        return

    state_names = list(named_states.keys())
    node.get_logger().info(f"Found {len(state_names)} poses: {state_names}")

    succeeded = 0
    failed = 0
    for name in state_names:
        ok = node.move_to_named_state(name, named_states[name])
        if ok:
            succeeded += 1
        else:
            failed += 1

    node.get_logger().info(
        f"=== Done! {succeeded}/{succeeded + failed} poses reached. ==="
    )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
