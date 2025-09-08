#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll/2.0), math.sin(roll/2.0)
    cp, sp = math.cos(pitch/2.0), math.sin(pitch/2.0)
    cy, sy = math.cos(yaw/2.0), math.sin(yaw/2.0)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return qx, qy, qz, qw


class MoveGroupClient(Node):
    def __init__(self):
        super().__init__("move_group_client")
        self.cli = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("Waiting for /move_action …")
        self.cli.wait_for_server()

    def plan_and_execute(self):
        qx, qy, qz, qw = quat_from_rpy(-math.pi/2, 0.0, 0.0)#(math.pi, 0.0, math.pi/2)

        # --- Goal constraints ---
        pos = PositionConstraint()
        pos.link_name = "tool0"
        pos.header.frame_id = "base_link"

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.02]  # 2 cm tolerance

        bv = BoundingVolume()
        p = PoseStamped()
        p.header.frame_id = "base_link"
        p.pose.position.x = 0.40
        p.pose.position.y = 0.0
        p.pose.position.z = 0.30
        bv.primitives.append(sphere)
        bv.primitive_poses.append(p.pose)
        pos.constraint_region = bv
        pos.weight = 1.0

        ori = OrientationConstraint()
        ori.link_name = "tool0"
        ori.header.frame_id = "base_link"
        ori.orientation.x = qx
        ori.orientation.y = qy
        ori.orientation.z = qz
        ori.orientation.w = qw
        ori.absolute_x_axis_tolerance = math.radians(5)
        ori.absolute_y_axis_tolerance = math.radians(5)
        ori.absolute_z_axis_tolerance = math.radians(5)
        ori.weight = 1.0

        goal = Constraints()
        goal.position_constraints.append(pos)
        goal.orientation_constraints.append(ori)

        req = MotionPlanRequest()
        req.group_name = "ur_manipulator"
        req.goal_constraints.append(goal)
        req.allowed_planning_time = 5.0

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options.plan_only = False

        self.get_logger().info("Sending goal to /move_action…")
        send_future = self.cli.send_goal_async(goal_msg, feedback_callback=None)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected ❌")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code.val == 1:
            self.get_logger().info("Motion complete ✅")
        else:
            self.get_logger().error(f"Execution failed with code {result.error_code.val}")


def main():
    rclpy.init()
    node = MoveGroupClient()
    node.plan_and_execute()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
