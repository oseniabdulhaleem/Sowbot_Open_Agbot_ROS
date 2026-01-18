import os
import subprocess

class ESPHandler:
    def __init__(self, node, serial_comm):
        self.node = node
        self.serial = serial_comm

    def enable_mcu(self):
        self.node.get_logger().info('Executing command: sudo /opt/lizard/espresso.py enable')
        if os.path.exists('/sys/class/gpio'):
            subprocess.run(["sudo", "/opt/lizard/espresso.py", "enable"])
        else:
            self.node.get_logger().info('Laptop detected: Skipping physical GPIO call')

    def send_esp_control(self, command):
        if os.path.exists('/sys/class/gpio'):
             subprocess.run(["sudo", "/opt/lizard/espresso.py", command])
        self.node.get_logger().info(f'ESP command simulated: {command}')
