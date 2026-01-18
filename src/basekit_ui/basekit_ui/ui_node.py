import sys
import os

# PATH INJECTION: Force Python to see ROS 2 Humble
ros_path = '/opt/ros/humble/lib/python3.10/site-packages'
if ros_path not in sys.path:
    sys.path.append(ros_path)

import rclpy
from nicegui import ui
import threading

class NiceGuiNode:
    def __init__(self):
        from rclcpp.node import Node
        self.node = Node('web_ui')
        
        @ui.page('/')
        def main_page():
            ui.label('AgBot Dashboard').classes('text-h4')
            ui.button('TEST LOG', on_click=lambda: self.node.get_logger().info('UI Click!'))

    def ros_main(self):
        rclpy.spin(self.node)

def main(args=None):
    rclpy.init(args=args)
    gui_node = NiceGuiNode()
    t = threading.Thread(target=gui_node.ros_main, daemon=True)
    t.start()
    ui.run(port=8080, show=False, reload=False)

if __name__ == '__main__':
    main()
