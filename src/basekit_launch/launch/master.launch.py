import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='basekit_driver',
            executable='basekit_driver_node',
            name='basekit_driver_node',
            parameters=[{'port': '/dev/ttyACM1', 'baudrate': 115200}]
        ),
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps_node',
            output='screen',
            parameters=[{
                'device': '/dev/ttyACM0',
                'baudrate': 115200,
                'uart1.baudrate': 115200,
                'frame_id': 'gps',
                'publish': {'all': True}
            }]
        ),
        Node(
            package='basekit_ui',
            executable='basekit_ui_node',
            name='basekit_ui_node'
        )
    ])
