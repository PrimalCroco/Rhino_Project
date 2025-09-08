import rclpy
from rclpy.node import Node
from moveit_py import MoveItPy
from geometry_msgs.msg import Pose

import math
from tf_transformations import quaternion_from_euler  # pip install transforms3d si nécessaire


class UR10CommanderIK(Node):
    def __init__(self):
        super().__init__("ur10_commander_ik")

        # Paramètres du robot (coordonnées et orientation)
        self.x = self.declare_parameter("x", 0.4).get_parameter_value().double_value
        self.y = self.declare_parameter("y", 0.0).get_parameter_value().double_value
        self.z = self.declare_parameter("z", 0.4).get_parameter_value().double_value
        self.alpha = self.declare_parameter("alpha", 0.0).get_parameter_value().double_value
        self.beta = self.declare_parameter("beta", 0.0).get_parameter_value().double_value
        self.gamma = self.declare_parameter("gamma", 0.0).get_parameter_value().double_value

        # Initialisation MoveItPy
        self.moveit = MoveItPy(node=self)

        # On envoie la trajectoire cartésienne
        self.execute_cartesian_path()

    def execute_cartesian_path(self):
        # Création de la pose
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = self.z

        # Conversion des angles Euler en quaternion
        q = quaternion_from_euler(self.alpha, self.beta, self.gamma)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        # Obtenir la trajectoire cartésienne
        plan = self.moveit.get_move_group("ur_manipulator").compute_cartesian_path([pose])

        if plan:
            self.get_logger().info("✅ Trajectoire cartésienne calculée, exécution...")
            success = self.moveit.get_move_group("ur_manipulator").execute(plan)
            if success:
                self.get_logger().info("✅ Trajectoire exécutée avec succès !")
            else:
                self.get_logger().warn("❌ La trajectoire n'a pas pu être exécutée")
        else:
            self.get_logger().error("❌ Impossible de calculer la trajectoire cartésienne")


def main(args=None):
    rclpy.init(args=args)
    node = UR10CommanderIK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

