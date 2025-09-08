import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusTcpClient
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from rcl_interfaces.msg import ParameterDescriptor
from example_interfaces.srv import SetBool  # ou crée ton propre service si tu veux plus complexe


from modbus_interface.srv import SetCoil  # <-- À créer (voir plus bas)

class ModbusROSNode(Node):
    def __init__(self):
        super().__init__('modbus_controller_node')

        # Paramètre IP (on ne choisit plus une seule adresse ici)
        self.declare_parameter(
            'plc_ip', '192.168.0.1',
            ParameterDescriptor(description='Adresse IP du PLC')
        )
        self.declare_parameter(
            'poll_rate', 1.0,
            ParameterDescriptor(description='Fréquence de lecture des coils (Hz)')
        )

        ip = self.get_parameter('plc_ip').get_parameter_value().string_value
        poll_rate = self.get_parameter('poll_rate').get_parameter_value().double_value

        # Connexion Modbus TCP
        self.client = ModbusTcpClient(ip)
        if not self.client.connect():
            self.get_logger().error(f"❌ Impossible de se connecter à {ip}:502")
            raise RuntimeError("Modbus connection failed")
        self.get_logger().info(f"✅ Connecté à {ip}:502")

        # Publier l'état de 8 coils (0 à 7)
        self.state_pubs = [
            self.create_publisher(Bool, f'coil_state/{i}', 10)
            for i in range(8)
        ]

        # Service pour changer n'importe quel coil
        self.srv = self.create_service(SetCoil, 'set_coil', self.set_coil_callback)

        # Timer de polling
        self.timer = self.create_timer(1.0 / poll_rate, self.poll_all_coils)

    def poll_all_coils(self):
        result = self.client.read_coils(address=0, count=8)
        if result.isError():
            self.get_logger().warn("⚠️ Erreur de lecture des coils")
            return

        for i in range(8):
            state = result.bits[i]
            msg = Bool()
            msg.data = state
            self.state_pubs[i].publish(msg)
            self.get_logger().debug(f"Coil[{i}] = {state}")

    def set_coil_callback(self, request, response):
        address = request.address
        value = request.state

        if not (0 <= address <= 7):
            response.success = False
            response.message = f"❌ Adresse {address} invalide (doit être entre 0 et 7)"
            return response

        result = self.client.write_coil(address=address, value=value)
        if result.isError():
            response.success = False
            response.message = "❌ Échec d’écriture du coil"
        else:
            response.success = True
            response.message = f"✅ Coil[{address}] mis à {'ON' if value else 'OFF'}"
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

