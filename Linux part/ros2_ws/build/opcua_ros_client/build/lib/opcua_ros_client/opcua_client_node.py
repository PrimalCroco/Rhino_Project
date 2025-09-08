from opcua import Client
import rclpy
from rclpy.node import Node

class OpcUaClientNode(Node):
    def __init__(self):
        super().__init__('opcua_ros_client')

        # Connexion OPC UA
        self.client = Client("opc.tcp://192.168.0.1:4840")  # Remplace par l’IP réelle de ton automate
        try:
            self.client.connect()
            self.get_logger().info("✅ Connecté au serveur OPC UA")

            # Récupère les noeuds OPC UA
            self.bool_node = self.client.get_node("ns=2;s=MyDB.BoolVar")
            self.int_node = self.client.get_node("ns=2;s=MyDB.IntVar")
            self.str_node = self.client.get_node("ns=2;s=MyDB.StringVar")

            # Timer ROS pour modifier les valeurs toutes les 2 secondes
            self.timer = self.create_timer(2.0, self.modify_values)
        except Exception as e:
            self.get_logger().error(f"❌ Erreur de connexion OPC UA : {e}")

    def modify_values(self):
        try:
            # Lecture (debug)
            bool_val = self.bool_node.get_value()
            int_val = self.int_node.get_value()
            str_val = self.str_node.get_value()

            self.get_logger().info(f"Avant écriture: Bool={bool_val}, Int={int_val}, Str={str_val}")

            # Écriture de nouvelles valeurs
            self.bool_node.set_value(not bool_val)
            self.int_node.set_value(int_val + 1)
            self.str_node.set_value("From ROS2")

            self.get_logger().info("✅ Valeurs écrites avec succès")
        except Exception as e:
            self.get_logger().error(f"Erreur d’accès OPC UA : {e}")

    def destroy_node(self):
        self.client.disconnect()
        super().destroy_node()

def main():
    rclpy.init()
    node = None
    try:
        node = OpcUaClientNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"❌ Erreur de connexion OPC UA : {e}")
    finally:
        if node:
            try:
                node.client.disconnect()
            except Exception as e:
                print(f"⚠️ Erreur lors de la déconnexion : {e}")
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
