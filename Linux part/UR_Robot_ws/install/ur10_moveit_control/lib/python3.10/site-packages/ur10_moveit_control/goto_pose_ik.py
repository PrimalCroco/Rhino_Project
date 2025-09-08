import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient

class URMover(Node):
    def __init__(self):
        super().__init__('ur10_mover')
        # Client IK
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        self.ik_client.wait_for_service()
        self.get_logger().info('Service compute_ik prêt')
        # Client action Traj
        self.traj_client = ActionClient(
    		self, FollowJointTrajectory,
    		'scaled_joint_trajectory_controller/follow_joint_trajectory')

        self.traj_client.wait_for_server()
        self.get_logger().info('Action follow_joint_trajectory prête')

    def goto(self, pose: PoseStamped):
        # 1. Appel IK
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'manipulator'
        req.ik_request.pose_stamped = pose
        req.ik_request.timeout.sec = 2
        res = self.ik_client.call(req)
        if res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().error('Échec IK')
            return

        # 2. Construire trajectoire
        js = res.solution.joint_state
        traj = JointTrajectory()
        traj.joint_names = js.name
        pt = JointTrajectoryPoint()
        pt.positions = js.position
        pt.time_from_start.sec = 5
        traj.points = [pt]

        # 3. Envoyer trajectoire
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self.traj_client.send_goal_async(goal)
        fut.result().result  # attend fin
        self.get_logger().info('Trajectoire exécutée')

def main():
    rclpy.init()
    mover = URMover()

    # Définir la cible
    pose = PoseStamped()
    pose.header.frame_id = 'base'
    pose.header.stamp = mover.get_clock().now().to_msg()
    pose.pose.position.x = 0.4
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.3
    pose.pose.orientation.w = 1.0

    mover.goto(pose)
    rclpy.spin(mover)
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
