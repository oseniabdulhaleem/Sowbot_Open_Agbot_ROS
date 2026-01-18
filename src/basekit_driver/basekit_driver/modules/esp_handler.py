import os
import subprocess

class ESPHandler:
    def __init__(self, node, serial_comm):
        self.node = node
        self.serial = serial_comm

    def enable_mcu(self):
        if os.path.exists('/sys/class/gpio'):
            subprocess.run(["sudo", "/opt/lizard/espresso.py", "enable"])
        else:
            self.node.get_logger().info('Laptop Mode: Skipping GPIO')

    def send_esp_control(self, command):
        if os.path.exists('/sys/class/gpio'):
             subprocess.run(["sudo", "/opt/lizard/espresso.py", command])
        self.node.get_logger().info(f'ESP Command: {command}')
