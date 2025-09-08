#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import numpy as np

from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll/2.0), math.sin(roll/2.0)
    cp, sp = math.cos(pitch/2.0), math.sin(pitch/2.0)
    cy, sy = math.cos(yaw/2.0), math.sin(yaw/2.0)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return qx, qy, qz, qw


def parametric_spiral_waypoints(center_x, center_y, z, ns, f_s_mm, tmax, time_step=0.1, roll=0, pitch=0, yaw=0, length_factor=2.0):
    """
    Generate a time-parameterized spiral with adjustable length using length_factor.
    """
    f_s = f_s_mm / 1000.0  # Convert mm → m
    beta = length_factor * np.sqrt(2 * np.pi * ns / 5)  # Factor to elongate the spiral

    t = np.arange(0, tmax + time_step, time_step)
    theta = beta * np.sqrt(t)
    r = f_s / np.pi * theta

    qx, qy, qz, qw = quat_from_rpy(roll, pitch, yaw)
    waypoints = []
    for ri, thetai in zip(r, theta):
        pose = Pose()
        pose.position.x = center_x + ri * np.cos(thetai)
        pose.position.y = center_y + ri * np.sin(thetai)
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        waypoints.append(pose)

    return waypoints


class SpiralTrajectoryClient(Node):
    def __init__(self):
        super().__init__("spiral_trajectory_client")

        # Service to compute the Cartesian path
        self.cartesian_srv = self.create_client(GetCartesianPath, "compute_cartesian_path")
        while not self.cartesian_srv.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for /compute_cartesian_path service...")

        # Action to execute a trajectory
        self.exec_action = ActionClient(self, ExecuteTrajectory, "execute_trajectory")
        self.get_logger().info("Waiting for /execute_trajectory action...")
        self.exec_action.wait_for_server()

    def plan_and_execute_spiral(self):
        # Generate configurable waypoints with a longer spiral
        waypoints = parametric_spiral_waypoints(
            center_x=0.4, center_y=0.0, z=0.3,
            ns=60, f_s_mm=0.54, tmax=4.7, time_step=0.1,
            roll=-math.pi/2, pitch=0.0, yaw=0.0,
            length_factor=2.5  # Adjust this factor to elongate the spiral
        )

        # Request for Cartesian path planning
        req = GetCartesianPath.Request()
        req.group_name = "ur_manipulator"
        req.link_name = "TCP"
        req.header.frame_id = "base_link"
        req.waypoints = waypoints
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        self.get_logger().info("Computing spiral trajectory via /compute_cartesian_path...")
        future = self.cartesian_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if not result:
            self.get_logger().error("Failed to call /compute_cartesian_path ❌")
            return

        if result.fraction < 0.99:
            self.get_logger().warn(f"Partial trajectory: {result.fraction*100:.1f}%")

        # Send trajectory to /execute_trajectory action
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = result.solution

        self.get_logger().info("Sending trajectory to /execute_trajectory...")
        send_future = self.exec_action.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected ❌")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        exec_result = result_future.result().result

        if exec_result.error_code.val == 1:
            self.get_logger().info("Spiral trajectory executed ✅")
        else:
            self.get_logger().error(f"Execution failed (code {exec_result.error_code.val})")


def main():
    rclpy.init()
    node = SpiralTrajectoryClient()
    node.plan_and_execute_spiral()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

