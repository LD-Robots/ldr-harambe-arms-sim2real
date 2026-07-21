#!/usr/bin/env python3
"""
Planning scene publisher for the MTC pick-and-place task.

Owns the collision scene: the object to pick and the two tables. Geometry comes
from config/mtc_task.yaml (scene.*), so this node and mtc_pick_place read the
same numbers rather than each carrying its own copy.

Objects are applied through the /apply_planning_scene SERVICE rather than the
/collision_object topic. The topic is fire-and-forget: a move_group that starts
late silently misses the objects, which is why this node used to re-publish on a
timer -- at the cost of move_group logging every update forever. The service
call is acknowledged, so waiting for it to appear and calling once is both
race-free and silent.

Usage:
    ros2 launch arm_mtc publish_planning_scene.launch.py
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

SERVICE = '/apply_planning_scene'


class PlanningScenePublisher(Node):
    """Applies the pick-and-place collision objects to the MoveIt planning scene."""

    def __init__(self):
        super().__init__('planning_scene_publisher')

        self.declare_parameter('robot.planning_frame', 'urdf_base')

        self.declare_parameter('scene.spawn_object', True)
        self.declare_parameter('scene.spawn_tables', True)
        self.declare_parameter('scene.republish_period', 0.0)

        self.declare_parameter('scene.object.id', 'target_cylinder')
        self.declare_parameter('scene.object.radius', 0.03035)
        self.declare_parameter('scene.object.height', 0.15)
        self.declare_parameter('scene.object.pose', [0.45, 0.35, 0.175])

        self.declare_parameter('scene.source_table.name', 'table')
        self.declare_parameter('scene.source_table.pose', [0.45, 0.35, -0.375])
        self.declare_parameter('scene.source_table.size', [0.5, 0.3, 0.95])

        self.declare_parameter('scene.destination_table.name', 'destination_table')
        self.declare_parameter('scene.destination_table.pose', [0.45, -0.35, -0.375])
        self.declare_parameter('scene.destination_table.size', [0.5, 0.3, 0.95])

        get = lambda name: self.get_parameter(name).value  # noqa: E731

        self.frame_id = get('robot.planning_frame')
        self.spawn_object = get('scene.spawn_object')
        self.spawn_tables = get('scene.spawn_tables')
        self.republish_period = get('scene.republish_period')

        self.object_id = get('scene.object.id')
        self.object_radius = get('scene.object.radius')
        self.object_height = get('scene.object.height')
        self.object_pose = list(get('scene.object.pose'))

        self.tables = [
            (get(f'scene.{key}.name'),
             list(get(f'scene.{key}.pose')),
             list(get(f'scene.{key}.size')))
            for key in ('source_table', 'destination_table')
        ]

        self.client = self.create_client(ApplyPlanningScene, SERVICE)
        self.applied = False
        self.log_scene()

        # Retry until move_group offers the service, then apply once.
        self.retry_timer = self.create_timer(1.0, self._try_apply)

    # ---------------------------------------------------------------- helpers

    def _collision_object(self, object_id, primitive, pose_xyz):
        obj = CollisionObject()
        obj.header = Header()
        obj.header.frame_id = self.frame_id
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = object_id
        obj.operation = CollisionObject.ADD

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pose_xyz
        pose.orientation.w = 1.0

        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        return obj

    def _scene_objects(self):
        objects = []
        if self.spawn_object:
            cylinder = SolidPrimitive()
            cylinder.type = SolidPrimitive.CYLINDER
            cylinder.dimensions = [self.object_height, self.object_radius]
            objects.append(self._collision_object(self.object_id, cylinder, self.object_pose))

        if self.spawn_tables:
            for name, pose, size in self.tables:
                box = SolidPrimitive()
                box.type = SolidPrimitive.BOX
                box.dimensions = list(size)
                objects.append(self._collision_object(name, box, pose))
        return objects

    def _apply(self, objects, done_callback):
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = objects
        self.client.call_async(request).add_done_callback(done_callback)

    # ---------------------------------------------------------------- applying

    def _try_apply(self):
        if not self.client.service_is_ready():
            self.get_logger().info(f'Waiting for {SERVICE} ...', throttle_duration_sec=5.0)
            return

        self.retry_timer.cancel()
        self._apply(self._scene_objects(), self._on_applied)

    def _on_applied(self, future):
        try:
            success = future.result().success
        except Exception as exc:  # noqa: BLE001 - report any transport failure and retry
            self.get_logger().error(f'{SERVICE} call failed: {exc}')
            self.retry_timer.reset()
            return

        if not success:
            self.get_logger().error(f'{SERVICE} rejected the scene diff; retrying')
            self.retry_timer.reset()
            return

        self.applied = True
        self.get_logger().info('Planning scene applied')

        # Optional slow re-assert, off by default. Only useful if something else
        # in the system clears the scene while this node keeps running.
        if self.republish_period > 0.0:
            self.get_logger().info(
                f'Re-asserting the scene every {self.republish_period:.1f} s')
            self.create_timer(self.republish_period,
                              lambda: self._apply(self._scene_objects(), lambda _f: None))

    # ---------------------------------------------------------------- logging

    def log_scene(self):
        self.get_logger().info(f"Planning scene in frame '{self.frame_id}':")
        if self.spawn_object:
            x, y, z = self.object_pose
            self.get_logger().info(
                f"  object '{self.object_id}': radius={self.object_radius}, "
                f"height={self.object_height} at ({x:.3f}, {y:.3f}, {z:.3f})")
        if self.spawn_tables:
            for name, pose, size in self.tables:
                self.get_logger().info(
                    f"  table '{name}': {size[0]}x{size[1]}x{size[2]} "
                    f"at ({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})")

    def clear_scene(self):
        if not self.applied:
            return
        removals = []
        for object_id in [self.object_id] + [name for name, _, _ in self.tables]:
            obj = CollisionObject()
            obj.header = Header()
            obj.header.frame_id = self.frame_id
            obj.id = object_id
            obj.operation = CollisionObject.REMOVE
            removals.append(obj)
        self._apply(removals, lambda _f: None)


def main(args=None):
    rclpy.init(args=args)
    node = PlanningScenePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.clear_scene()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
