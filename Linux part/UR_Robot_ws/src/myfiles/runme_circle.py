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


def circular_waypoints(center_x, center_y, z, radius, n_points, roll=0, pitch=0, yaw=0):
    waypoints = []
    qx, qy, qz, qw = quat_from_rpy(roll, pitch, yaw)
    for theta in np.linspace(0, 2*math.pi, n_points, endpoint=True):
        pose = Pose()
        pose.position.x = center_x + radius * math.cos(theta)
        pose.position.y = center_y + radius * math.sin(theta)
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        waypoints.append(pose)
    return waypoints


class CircularTrajectoryClient(Node):
    def __init__(self):
        super().__init__("circular_trajectory_client")

        # Service pour calculer le chemin cartésien
        self.cartesian_srv = self.create_client(GetCartesianPath, "compute_cartesian_path")
        while not self.cartesian_srv.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Attente du service /compute_cartesian_path...")

        # Action pour exécuter une trajectoire
        self.exec_action = ActionClient(self, ExecuteTrajectory, "execute_trajectory")
        self.get_logger().info("Attente de l’action /execute_trajectory...")
        self.exec_action.wait_for_server()

    def plan_and_execute_circle(self):
        waypoints = circular_waypoints(
            center_x=0.4, center_y=0.0, z=0.3,
            radius=0.1, n_points=50,
            roll=-math.pi/2, pitch=0.0, yaw=0.0
        )

        # Requête pour plan cartésien
        req = GetCartesianPath.Request()
        req.group_name = "ur_manipulator"
        req.link_name = "tool0"
        req.header.frame_id = "base_link"
        req.waypoints = waypoints
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        self.get_logger().info("Calcul de la trajectoire circulaire via /compute_cartesian_path...")
        future = self.cartesian_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if not result:
            self.get_logger().error("Échec de /compute_cartesian_path ❌")
            return

        if result.fraction < 0.99:
            self.get_logger().warn(f"Trajectoire partielle : {result.fraction*100:.1f}%")

        # Envoi de la trajectoire à l’action /execute_trajectory
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = result.solution

        self.get_logger().info("Envoi de la trajectoire à /execute_trajectory...")
        send_future = self.exec_action.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejeté ❌")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        exec_result = result_future.result().result

        if exec_result.error_code.val == 1:
            self.get_logger().info("Trajectoire circulaire exécutée ✅")
        else:
            self.get_logger().error(f"Échec exécution (code {exec_result.error_code.val})")


def main():
    rclpy.init()
    node = CircularTrajectoryClient()
    node.plan_and_execute_circle()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

