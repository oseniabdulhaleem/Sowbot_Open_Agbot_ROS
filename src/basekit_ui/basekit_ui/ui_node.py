import rclpy
from nicegui import ui
import threading

# We import Node inside the class or method to avoid top-level import issues
class NiceGuiNode:
    def __init__(self):
        from rclcpp.node import Node
        self.node = Node('web_ui')
        
        @ui.page('/')
        def main_page():
            ui.label('AgBot Dashboard').classes('text-h4 q-pa-md')
            with ui.row():
                ui.button('STOP', color='red', on_click=lambda: self.node.get_logger().error('ESTOP'))

    def ros_main(self):
        rclpy.spin(self.node)

def main(args=None):
    rclpy.init(args=args)
    gui_node = NiceGuiNode()
    threading.Thread(target=gui_node.ros_main, daemon=True).start()
    ui.run(port=8080, show=False)

if __name__ == '__main__':
    main()
