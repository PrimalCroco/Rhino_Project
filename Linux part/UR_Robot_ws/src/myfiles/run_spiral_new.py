#!/usr/bin/env python3
"""
run_spiral.py — UR + MoveIt (ROS 2 Humble)

Pipeline:
  A) Build helical spiral (XY spiral + Z ramp).
  B) JOINT-SPACE approach to a lifted pre-approach pose (first waypoint + lift in Z),
     using collision-aware IK and joint-space planning.
  C) Short CARTESIAN drop from pre-approach → first waypoint.
  D) One CARTESIAN sweep across the full spiral.
  E) RViz markers for spiral and approach.

Usage:
  source ~/ur_ws/install/setup.bash
  ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10 launch_rviz:=true use_fake_hardware:=true
  # new terminal:
  python3 ~/ur_ws/src/myfiles/run_spiral.py \
    --points 40 --r-start 0.02 --r-end 0.08 --z-span 0.03 \
    --max-step 0.005 --approach-lift 0.10 --waypoint-frame base_link --marker-frame world --tip TCP
"""

import math
import argparse
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Pose, PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

from moveit_msgs.msg import RobotTrajectory, Constraints, JointConstraint, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetCartesianPath, GetPositionIK, GetMotionPlan
from moveit_msgs.action import ExecuteTrajectory

from tf_transformations import quaternion_from_euler


# ---------- Geometry helpers ----------

def make_helical_spiral(cx, cy, z_start, z_end, r_start, r_end, turns, n_points, rx, ry, rz) -> List[Pose]:
    assert n_points >= 2
    poses: List[Pose] = []
    total_theta = 2.0 * math.pi * turns
    k_r = (r_end - r_start) / max(total_theta, 1e-9)
    qx, qy, qz, qw = quaternion_from_euler(rx, ry, rz)

    for i in range(int(n_points)):
        s = i / (n_points - 1)
        theta = s * total_theta
        r = r_start + k_r * theta
        z = z_start + s * (z_end - z_start)

        p = Pose()
        p.position.x = cx + r * math.cos(theta)
        p.position.y = cy + r * math.sin(theta)
        p.position.z = z
        p.orientation.x = qx
        p.orientation.y = qy
        p.orientation.z = qz
        p.orientation.w = qw
        poses.append(p)
    return poses


# ---------- Main node ----------

class SpiralPlanner(Node):
    def __init__(self, args):
        super().__init__("spiral_cartesian_planner")

        # CLI params
        self.group = args.group
        self.tip = args.tip
        self.waypoint_frame = args.waypoint_frame
        self.marker_frame = args.marker_frame

        self.max_step = float(args.max_step)
        self.jump_thresh = float(args.jump_thresh)
        self.avoid_collisions = bool(args.avoid_collisions)
        self.joint_tol = float(args.joint_tolerance)
        self.planning_time = float(args.planning_time)
        self.approach_lift = float(args.approach_lift)
        self.ik_collisions = not args.relax_ik  # default True (collision-aware IK)

        # Spiral waypoints (first Cartesian target at index 0)
        self.waypoints = make_helical_spiral(
            cx=args.center_x, cy=args.center_y,
            z_start=args.center_z, z_end=args.center_z + args.z_span,
            r_start=args.r_start, r_end=args.r_end,
            turns=args.turns, n_points=args.points,
            rx=args.rx, ry=args.ry, rz=args.rz,
        )

        # Pre-approach pose (lifted above first waypoint to avoid self-collision)
        self.pre_approach = Pose()
        self.pre_approach.position.x = self.waypoints[0].position.x
        self.pre_approach.position.y = self.waypoints[0].position.y
        self.pre_approach.position.z = self.waypoints[0].position.z + self.approach_lift
        self.pre_approach.orientation = self.waypoints[0].orientation

        # Services / action
        self.scene_cli = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.cart_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.exec_ac = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

        # RViz markers
        self.spiral_pub = self.create_publisher(Marker, "spiral_marker", 1)
        self.approach_pub = self.create_publisher(Marker, "approach_marker", 1)

    # ----- Helpers -----

    def wait_for_endpoints(self, timeout_sec=20.0):
        for name, cli in [
            ("/get_planning_scene", self.scene_cli),
            ("/compute_ik", self.ik_cli),
            ("/plan_kinematic_path", self.plan_cli),
            ("/compute_cartesian_path", self.cart_cli),
        ]:
            self.get_logger().info(f"Waiting for {name} …")
            if not cli.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().error(f"Service {name} not available")
                return False
        self.get_logger().info("Waiting for /execute_trajectory action …")
        if not self.exec_ac.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error("Action /execute_trajectory not available")
            return False
        return True

    def get_robot_state(self):
        req = GetPlanningScene.Request()
        req.components = PlanningSceneComponents(components=PlanningSceneComponents.ROBOT_STATE)
        fut = self.scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        if not res:
            raise RuntimeError("Failed to get planning scene / robot state")
        return res.scene.robot_state

    # ----- Visualization -----

    def publish_spiral_marker(self):
        m = Marker()
        m.header.frame_id = self.marker_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "spiral"
        m.id = 1
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.004
        m.color = ColorRGBA(r=0.0, g=0.7, b=1.0, a=1.0)
        for p in self.waypoints:
            m.points.append(Point(x=p.position.x, y=p.position.y, z=p.position.z))
        self.spiral_pub.publish(m)
        self.get_logger().info(f"Published spiral marker with {len(self.waypoints)} points")

    def publish_approach_marker(self):
        m = Marker()
        m.header.frame_id = self.marker_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "approach"
        m.id = 2
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.02
        m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
        m.pose = self.pre_approach
        self.approach_pub.publish(m)
        self.get_logger().info("Published pre-approach marker")

    # ----- Planning blocks -----

    def compute_ik(self, pose: Pose) -> Tuple[list, list]:
        pose_st = PoseStamped()
        pose_st.header.frame_id = self.waypoint_frame
        pose_st.pose = pose

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group
        req.ik_request.ik_link_name = self.tip
        req.ik_request.pose_stamped = pose_st
        req.ik_request.avoid_collisions = self.ik_collisions  # collision-aware IK
        req.ik_request.robot_state = self.get_robot_state()
        req.ik_request.timeout = DurationMsg(sec=1, nanosec=0)

        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if not res or res.error_code.val != 1:
            raise RuntimeError(f"IK failed (code={None if not res else res.error_code.val})")

        jstate = res.solution.joint_state
        return list(jstate.name), list(jstate.position)

    def plan_to_joint_goal(self, joint_names: list, positions: list) -> RobotTrajectory:
        # joint goal with small tolerances
        jcs = []
        for name, pos in zip(joint_names, positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = self.joint_tol
            jc.tolerance_below = self.joint_tol
            jc.weight = 1.0
            jcs.append(jc)

        constraints = Constraints(joint_constraints=jcs)

        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = self.group
        req.motion_plan_request.start_state = self.get_robot_state()
        req.motion_plan_request.goal_constraints = [constraints]
        req.motion_plan_request.allowed_planning_time = self.planning_time

        fut = self.plan_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        res = fut.result()
        if not res or res.motion_plan_response.error_code.val != 1:
            code = None if not res else res.motion_plan_response.error_code.val
            raise RuntimeError(f"Joint-space plan failed (code={code})")

        traj = res.motion_plan_response.trajectory
        if not isinstance(traj, RobotTrajectory):
            raise RuntimeError("Joint-space plan returned no trajectory")
        return traj

    def execute_traj(self, traj: RobotTrajectory) -> bool:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj
        send = self.exec_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        gh = send.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("ExecuteTrajectory goal rejected")
            return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        res = res_fut.result()
        ok = res and getattr(res.result.error_code, "val", 0) == 1
        self.get_logger().info("Execution OK ✅" if ok else "Execution failed ❌")
        return bool(ok)

    def compute_cartesian(self, waypoints: List[Pose]) -> RobotTrajectory:
        req = GetCartesianPath.Request()
        req.group_name = self.group
        req.link_name = self.tip
        req.waypoints = waypoints
        req.max_step = self.max_step
        req.jump_threshold = self.jump_thresh
        req.avoid_collisions = self.avoid_collisions
        req.start_state = self.get_robot_state()

        self.get_logger().info(f"Computing Cartesian path over {len(waypoints)} waypoints…")
        fut = self.cart_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        res = fut.result()
        if not res:
            raise RuntimeError("GetCartesianPath failed / timed out")
        self.get_logger().info(f"Cartesian fraction: {res.fraction:.3f}")
        if res.fraction < 0.99:
            raise RuntimeError(f"Cartesian path incomplete: fraction={res.fraction:.3f}")
        return res.solution


# ---------- CLI ----------

def parse_args():
    ap = argparse.ArgumentParser(description="Spiral with joint-space pre-approach + Cartesian sweep")

    # MoveIt / frames
    ap.add_argument("--group", default="ur_manipulator")
    ap.add_argument("--tip", default="TCP")
    ap.add_argument("--waypoint-frame", default="base_link")
    ap.add_argument("--marker-frame", default="world")

    # Spiral geometry
    ap.add_argument("--center-x", type=float, default=0.40)
    ap.add_argument("--center-y", type=float, default=0.00)
    ap.add_argument("--center-z", type=float, default=0.30)
    ap.add_argument("--z-span", type=float, default=0.02)
    ap.add_argument("--r-start", type=float, default=0.02)
    ap.add_argument("--r-end", type=float, default=0.08)
    ap.add_argument("--turns", type=float, default=1.25)
    ap.add_argument("--points", type=int, default=25)

    # Orientation (TCP down by default)
    ap.add_argument("--rx", type=float, default=math.pi)
    ap.add_argument("--ry", type=float, default=0.0)
    ap.add_argument("--rz", type=float, default=0.0)

    # Planning options
    ap.add_argument("--approach-lift", type=float, default=0.10, help="Lift above first waypoint [m]")
    ap.add_argument("--max-step", type=float, default=0.005)
    ap.add_argument("--jump-thresh", type=float, default=0.0)
    ap.add_argument("--avoid-collisions", action="store_true")
    ap.add_argument("--joint-tolerance", type=float, default=0.01)
    ap.add_argument("--planning-time", type=float, default=2.0)
    ap.add_argument("--relax-ik", action="store_true", help="If set, IK ignores collisions")

    return ap.parse_args()


# ---------- Entry ----------

def main():
    args = parse_args()
    rclpy.init()
    node = SpiralPlanner(args)

    if not node.wait_for_endpoints():
        node.destroy_node()
        rclpy.shutdown()
        return

    # Markers for RViz
    node.publish_spiral_marker()
    node.publish_approach_marker()

    try:
        # (A) JOINT approach to lifted pre-approach
        jn, jp = node.compute_ik(node.pre_approach)
        node.get_logger().info("IK to pre-approach OK — planning joint-space approach…")
        jtraj = node.plan_to_joint_goal(jn, jp)
        if not node.execute_traj(jtraj):
            raise RuntimeError("Failed to execute joint-space approach")

        # (B) Short Cartesian drop to first waypoint
        drop_traj = node.compute_cartesian([node.pre_approach, node.waypoints[0]])
        if not node.execute_traj(drop_traj):
            raise RuntimeError("Failed to execute drop to first waypoint")

        # (C) Full Cartesian spiral
        spiral_traj = node.compute_cartesian(node.waypoints)
        node.execute_traj(spiral_traj)

    except Exception as e:
        node.get_logger().error(str(e))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()



