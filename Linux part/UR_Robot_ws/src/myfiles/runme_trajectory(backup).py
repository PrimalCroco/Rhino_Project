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


class CartesianWaypoints(Node):
    def __init__(self):
        super().__init__("cartesian_waypoints")
        self.cli = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("⏳ Attente du serveur /move_action…")
        self.cli.wait_for_server()

    def send_goal(self, x, y, z, roll, pitch, yaw):
        """Construit et envoie une contrainte pour un waypoint"""
        qx, qy, qz, qw = quat_from_rpy(roll, pitch, yaw)

        # Position constraint
        pos = PositionConstraint()
        pos.link_name = "tool0"
        pos.header.frame_id = "base_link"

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # tolérance 1 cm

        bv = BoundingVolume()
        p = PoseStamped()
        p.header.frame_id = "base_link"
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        bv.primitives.append(sphere)
        bv.primitive_poses.append(p.pose)
        pos.constraint_region = bv
        pos.weight = 1.0

        # Orientation constraint
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

        self.get_logger().info(f"🚀 Envoi du waypoint : ({x:.2f}, {y:.2f}, {z:.2f})")
        send_future = self.cli.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Waypoint rejeté")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code.val == 1:
            self.get_logger().info("✅ Waypoint atteint")
        else:
            self.get_logger().warn(f"⚠️ Échec avec code {result.error_code.val}")

    def run_trajectory(self, waypoints):
        for (x, y, z, r, p, yaw) in waypoints:
            self.send_goal(x, y, z, r, p, yaw)


def main():
    rclpy.init()
    node = CartesianWaypoints()

    # Liste de waypoints
    waypoints = [
	(0.0, 0.0, 0.3, -1.5707963267948966, 0.0, 0.0) ,
	(-0.4355060787535308, 0.18193528001898773, 0.30425531914893617, -1.5707963267948966, 0.0, 0.0) ,
	(-0.4921669087909796, -0.45089213275999807, 0.30851063829787234, -1.5707963267948966, 0.0, 0.0) ,
	(0.035634572282937114, -0.8167179130512594, 0.3127659574468085, -1.5707963267948966, 0.0, 0.0) ,
	(0.6634382502030672, -0.6715010433014594, 0.3170212765957447, -1.5707963267948966, 0.0, 0.0) ,
	(1.0445751774369005, -0.1506408059490804, 0.3212765957446808, -1.5707963267948966, 0.0, 0.0) ,
	(1.044609671006171, 0.4953650973994471, 0.325531914893617, -1.5707963267948966, 0.0, 0.0) ,
	(0.6938004523536135, 1.0382691691184756, 0.32978723404255317, -1.5707963267948966, 0.0, 0.0) ,
	(0.11663104370420353, 1.3298590623142665, 0.33404255319148934, -1.5707963267948966, 0.0, 0.0) ,
	(-0.5299843257945497, 1.313015815264185, 0.3382978723404255, -1.5707963267948966, 0.0, 0.0) ,
	(-1.100615716512141, 1.0081195127737121, 0.3425531914893617, -1.5707963267948966, 0.0, 0.0) ,
	(-1.4870241149558643, 0.4890656190688869, 0.3468085106382979, -1.5707963267948966, 0.0, 0.0) ,
	(-1.6287766358299913, -0.14240280733435404, 0.35106382978723405, -1.5707963267948966, 0.0, 0.0) ,
	(-1.5128797627094934, -0.7792000406142824, 0.3553191489361702, -1.5707963267948966, 0.0, 0.0) ,
	(-1.1663987783805774, -1.3259855807090792, 0.3595744680851064, -1.5707963267948966, 0.0, 0.0) ,
	(-0.6454041925456808, -1.7102465580867041, 0.3638297872340425, -1.5707963267948966, 0.0, 0.0) ,
	(-0.0228046067550183, -1.8877859523422071, 0.3680851063829787, -1.5707963267948966, 0.0, 0.0) ,
	(0.623140788878658, -1.8435610648775302, 0.3723404255319149, -1.5707963267948966, 0.0, 0.0) ,
	(1.218507451793399, -1.589033487176249, 0.37659574468085105, -1.5707963267948966, 0.0, 0.0) ,
	(1.7010094153755015, -1.1572038683802919, 0.38085106382978723, -1.5707963267948966, 0.0, 0.0) ,
	(2.0247554540411667, -0.5963934659835147, 0.3851063829787234, -1.5707963267948966, 0.0, 0.0) ,
	(2.162582956427229, 0.036342292918993296, 0.3893617021276596, -1.5707963267948966, 0.0, 0.0) ,
	(2.1062833965698315, 0.6814851794171788, 0.3936170212765957, -1.5707963267948966, 0.0, 0.0) ,
	(1.8651376555953432, 1.282528484228784, 0.3978723404255319, -1.5707963267948966, 0.0, 0.0) ,
	(1.463218913817143, 1.7903558831060709, 0.40212765957446805, -1.5707963267948966, 0.0, 0.0) ,
	(0.935912857482166, 2.1663833667823362, 0.40638297872340423, -1.5707963267948966, 0.0, 0.0) ,
	(0.3260645898318227, 2.384449127718239, 0.4106382978723404, -1.5707963267948966, 0.0, 0.0) ,
	(-0.3198986460859948, 2.431531740775283, 0.4148936170212766, -1.5707963267948966, 0.0, 0.0) ,
	(-0.9555901472611469, 2.307443436223218, 0.41914893617021276, -1.5707963267948966, 0.0, 0.0) ,
	(-1.5378258782757224, 2.023686067492822, 0.42340425531914894, -1.5707963267948966, 0.0, 0.0) ,
	(-2.029189971527248, 1.6016765534461592, 0.4276595744680851, -1.5707963267948966, 0.0, 0.0) ,
	(-2.3999307493992132, 1.0705503777220928, 0.4319148936170213, -1.5707963267948966, 0.0, 0.0) ,
	(-2.629168720104005, 0.46474029858225063, 0.43617021276595747, -1.5707963267948966, 0.0, 0.0) ,
	(-2.7054422547586587, -0.17849341952268769, 0.44042553191489364, -1.5707963267948966, 0.0, 0.0) ,
	(-2.6266506118694357, -0.8214318283091394, 0.4446808510638298, -1.5707963267948966, 0.0, 0.0) ,
	(-2.3994780273740726, -1.4280458228954955, 0.44893617021276594, -1.5707963267948966, 0.0, 0.0) ,
	(-2.038397568249911, -1.9658359597484105, 0.4531914893617021, -1.5707963267948966, 0.0, 0.0) ,
	(-1.5643604713269512, -2.407305126772194, 0.4574468085106383, -1.5707963267948966, 0.0, 0.0) ,
	(-1.0032770231195078, -2.7310332958902745, 0.46170212765957447, -1.5707963267948966, 0.0, 0.0) ,
	(-0.38438999724949013, -2.9223480232755024, 0.4659574468085106, -1.5707963267948966, 0.0, 0.0) ,
	(0.2613674577049931, -2.9736049976295424, 0.47021276595744677, -1.5707963267948966, 0.0, 0.0) ,
	(0.9029495884085792, -2.884109509261777, 0.47446808510638294, -1.5707963267948966, 0.0, 0.0) ,
	(1.510645364284815, -2.6597222025491853, 0.4787234042553191, -1.5707963267948966, 0.0, 0.0) ,
	(2.0573438948662415, -2.312201051027301, 0.4829787234042553, -1.5707963267948966, 0.0, 0.0) ,
	(2.5195811496681837, -1.8583364666009146, 0.48723404255319147, -1.5707963267948966, 0.0, 0.0) ,
	(2.878345277769861, -1.3189382265856207, 0.49148936170212765, -1.5707963267948966, 0.0, 0.0) ,
	(3.1196308180248824, -0.7177319371361078, 0.4957446808510638, -1.5707963267948966, 0.0, 0.0) ,
	(3.2347436360108226, -0.08021953870532833, 0.5, -1.5707963267948966, 0.0, 0.0) 
    ]

    node.run_trajectory(waypoints)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

