import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusTcpClient

class ModbusConnectionChecker(Node):
    def __init__(self):
        super().__init__('modbus_connection_checker')

        self.declare_parameter('plc_ip', '192.168.0.1')
        self.declare_parameter('unit_id', 1)

        ip = self.get_parameter('plc_ip').get_parameter_value().string_value
        unit_id = self.get_parameter('unit_id').get_parameter_value().integer_value

        self.client = ModbusTcpClient(ip)

        if self.client.connect():
            self.get_logger().info(f'✅ Connexion Modbus réussie au PLC @ {ip}, unité {unit_id}')
            self.client.close()
        else:
            self.get_logger().error(f'❌ Connexion Modbus échouée au PLC @ {ip}, unité {unit_id}')

def main(args=None):
    rclpy.init(args=args)
    node = ModbusConnectionChecker()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

