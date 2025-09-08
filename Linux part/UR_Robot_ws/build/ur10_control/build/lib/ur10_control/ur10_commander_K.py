#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class UR10Commander(Node):
    def __init__(self):
        super().__init__('ur10_commander')
        self.publisher = self.create_publisher(JointTrajectory,
                                               '/scaled_joint_trajectory_controller/joint_trajectory',
                                               10)

        self.timer = self.create_timer(2.0, self.send_goal)

    def send_goal(self):
        traj = JointTrajectory()
        traj.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        point = JointTrajectoryPoint()
        # Exemple : robot droit
        point.positions = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 5  # durée de la trajectoire en secondes

        traj.points.append(point)
        self.publisher.publish(traj)
        self.get_logger().info("Trajectory command sent.")

        # on arrête après une commande
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = UR10Commander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
