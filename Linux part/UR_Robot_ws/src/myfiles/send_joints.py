#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

UR10_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

class URJointCommander(Node):
    def __init__(self, action_ns="scaled_joint_trajectory_controller/follow_joint_trajectory"):
        super().__init__("ur_joint_commander")
        self._client = ActionClient(self, FollowJointTrajectory, action_ns)
        self.get_logger().info(f"Waiting for action server: {action_ns}")
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"Action server not available: {action_ns}")

    def send(self, positions, time_sec=3.0):
        if len(positions) != len(UR10_JOINTS):
            raise ValueError(f"Expected {len(UR10_JOINTS)} positions, got {len(positions)}")

        traj = JointTrajectory()
        traj.joint_names = UR10_JOINTS

        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start.sec = int(time_sec)
        pt.time_from_start.nanosec = int((time_sec - int(time_sec)) * 1e9)
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance.sec = 1  # small cushion

        self.get_logger().info(f"Sending trajectory to {traj.joint_names}")
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("FollowJointTrajectory goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        ok = result and result.result and result.result.error_code == 0
        self.get_logger().info("Execution OK ✅" if ok else f"Execution failed, code={getattr(result.result, 'error_code', 'n/a')}")
        return ok

def main():
    rclpy.init()
    # Use scaled controller by default; switch below to 'joint_trajectory_controller/follow_joint_trajectory' if needed.
    node = URJointCommander("joint_trajectory_controller/follow_joint_trajectory")
    try:
        # Example 1: small move from zeros (radians)
        node.send([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], time_sec=4.0)

        # Example 2: another pose
        node.send([0.2, -1.2, 1.3, -1.6, -1.5, 0.3], time_sec=4.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()