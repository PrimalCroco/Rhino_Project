import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusTcpClient
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class ModbusROSNode(Node):
    def __init__(self):
        super().__init__('modbus_controller_node')

        # Paramètres (modifiable via ros2 param)
        self.declare_parameter('plc_ip', '192.168.0.1')
        self.declare_parameter('coil_address', 1)
        self.declare_parameter('poll_rate', 1.0)  # en Hz

        ip = self.get_parameter('plc_ip').get_parameter_value().string_value
        self.coil_address = self.get_parameter('coil_address').get_parameter_value().integer_value
        poll_rate = self.get_parameter('poll_rate').get_parameter_value().double_value

        self.client = ModbusTcpClient(ip)
        if not self.client.connect():
            self.get_logger().error(f"❌ Impossible de se connecter à {ip}:502")
            raise RuntimeError("Modbus connection failed")

        self.get_logger().info(f"✅ Connecté à {ip}:502")

        # Publisher pour l'état du Coil
        self.state_pub = self.create_publisher(Bool, 'coil_state', 10)

        # Service pour changer l'état du Coil
        self.srv = self.create_service(SetBool, 'set_coil', self.set_coil_callback)

        # Timer pour polling
        self.timer = self.create_timer(1.0 / poll_rate, self.poll_coil_state)

    def poll_coil_state(self):
        result = self.client.read_coils(address=self.coil_address, count=1)
        if result.isError():
            self.get_logger().warn("⚠️ Erreur de lecture du Coil")
            return

        coil_state = result.bits[0]
        msg = Bool()
        msg.data = coil_state
        self.state_pub.publish(msg)
        self.get_logger().debug(f"Coil[{self.coil_address}] = {coil_state}")

    def set_coil_callback(self, request, response):
        value = request.data
        result = self.client.write_coil(address=self.coil_address, value=value)

        if result.isError():
            response.success = False
            response.message = "❌ Échec de l’écriture du Coil"
        else:
            response.success = True
            response.message = f"✅ Coil mis à {'ON' if value else 'OFF'}"
            self.get_logger().info(response.message)

        return response

    def destroy_node(self):
        self.client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ModbusROSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Arrêt du nœud demandé par l'utilisateur")
    finally:
        node.destroy_node()
        rclpy.shutdown()

