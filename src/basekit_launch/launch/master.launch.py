import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('basekit_launch'),
        'config',
        'thinkpad_override.yaml'
    )

    return LaunchDescription([
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps_node',
            parameters=[{'device': '/dev/ttyACM0', 'tmode3': 0}]
        ),
        Node(
            package='basekit_driver',
            executable='basekit_driver_node',
            name='basekit_driver_node',
            parameters=[config]
        ),
        Node(package='basekit_ui', executable='basekit_ui_node', name='basekit_ui_node')
    ])
