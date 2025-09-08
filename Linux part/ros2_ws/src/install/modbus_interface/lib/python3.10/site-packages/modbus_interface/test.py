import rclpy
from rclpy.node import Node

from pymodbus.client import ModbusTcpClient


class ModbusTcpRos2Node(Node):

    def __init__(self):
        super().__init__('modbus_tcp_ros2_node')
        self.plc_ip = '192.168.0.1'
        self.plc_port = 502  # changer ici avec le port que tu as configuré dans TIA Portal
        
        # Création client Modbus TCP
        self.client = ModbusTcpClient(self.plc_ip, port=self.plc_port)
        
        connected = self.client.connect()
        if connected:
            self.get_logger().info(f'Connecté au PLC Modbus TCP sur {self.plc_ip}:{self.plc_port}')
        else:
            self.get_logger().error(f'Impossible de se connecter au PLC Modbus TCP sur {self.plc_ip}:{self.plc_port}')
            return
        
        # Timer pour lire les registres toutes les 2 secondes
        self.timer = self.create_timer(2.0, self.read_registers)

    def read_registers(self):
        # Exemple lecture de 2 registres à l'adresse 0
        result = self.client.read_holding_registers(0, 2, unit=1)  # unit = ID esclave Modbus
        if not result.isError():
            self.get_logger().info(f"Registres lus: {result.registers}")
        else:
            self.get_logger().error("Erreur lecture registre Modbus")

    def destroy_node(self):
        self.client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ModbusTcpRos2Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

