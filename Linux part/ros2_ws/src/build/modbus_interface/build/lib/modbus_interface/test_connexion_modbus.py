from pymodbus.client import ModbusTcpClient

ip = '192.168.0.1'
client = ModbusTcpClient(ip)

if client.connect():
    print(f"✅ Connexion réussie au PLC @ {ip}")
    client.close()
else:
    print(f"❌ Connexion échouée au PLC @ {ip}")
