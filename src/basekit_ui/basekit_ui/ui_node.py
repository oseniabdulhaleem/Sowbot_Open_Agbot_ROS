import rclpy
from rclpy.node import Node
from nicegui import ui
import threading
import sys

class NiceGuiNode(Node):
    def __init__(self):
        super().__init__('web_ui')
        
        @ui.page('/')
        def main_page():
            ui.dark_mode().enable()
            with ui.column().classes('w-full items-center q-pa-md'):
                ui.label('AgBot Control Panel').classes('text-h3 text-primary font-bold')
                
                with ui.card().classes('q-pa-lg bg-grey-9'):
                    ui.label('System Status').classes('text-h5 text-white')
                    ui.separator()
                    ui.label('GPS: Connected').classes('text-green text-subtitle1')
                    ui.label('MCU: Connected').classes('text-green text-subtitle1')
                    
                ui.button('TEST ROS LOG', 
                          on_click=lambda: self.get_logger().info('NiceGUI interaction success!'),
                          color='primary').classes('q-mt-md')

def main(args=None):
    rclpy.init(args=args)
    node = NiceGuiNode()
    
    # Run ROS in background
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    
    # Start web server
    ui.run(port=8080, show=False, reload=False)

if __name__ == '__main__':
    main()
