#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown

def main():
    rclpy.init()
    roscpp_initialize([])
    node = Node('go_to_pose_node')

    robot = RobotCommander()
    scene = PlanningSceneInterface()
    group_name = "manipulator"  # changer selon ton MoveIt config
    move_group = MoveGroupCommander(group_name)

    # exemple: aller à une position pré-définie
    pose_goal = move_group.get_current_pose().pose
    pose_goal.position.x += 0.1
    move_group.set_pose_target(pose_goal)
    move_group.go(wait=True)

    move_group.stop()
    move_group.clear_pose_targets()

    roscpp_shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

