# ros_connection.py
import roslibpy

LIMO_IP = "192.168.0.105" 
LIMO_PORT = 9090  # Wichtig: 9090 für die Bridge!

# Globaler Client, den alle Tools importieren können
client = roslibpy.Ros(host=LIMO_IP, port=LIMO_PORT)