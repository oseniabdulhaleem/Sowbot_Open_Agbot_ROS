import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # 1. Config Paths
    try:
        ublox_config = os.path.join(get_package_share_directory('ublox_gps'), 'config', 'zed_f9p.yaml')
    except PackageNotFoundError:
        ublox_config = ""

    # 2. Hardware Nodes (GPS & MCU)
    gps_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='screen',
        respawn=True,
        parameters=[ublox_config, {
            'device': LaunchConfiguration('gps_port', default='/dev/ttyACM0'),
            'uart1.baudrate': 460800,
            'meas_rate': 1000,
            'tmode3': 0,
        }]
    )

    mcu_node = Node(
        package='basekit_driver',
        executable='basekit_driver_node',
        name='basekit_driver_node',
        parameters=[{'port': LaunchConfiguration('mcu_port', default='/dev/ttyACM1')}],
        respawn=True
    )

    ld.add_action(gps_node)
    ld.add_action(mcu_node)

    # 3. Conditional Web UI Launch (Prevents the crash you just saw)
    try:
        rosbridge_dir = get_package_share_directory('rosbridge_server')
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(rosbridge_dir, 'launch/rosbridge_websocket_launch.xml')])
        ))
        ld.add_action(Node(package='web_ui', executable='web_ui_node', name='web_ui_node'))
        ld.add_action(LogInfo(msg="✅ Web UI and Bridge found and starting."))
    except PackageNotFoundError:
        ld.add_action(LogInfo(msg="⚠️ rosbridge_server not found. Web UI will be disabled, but hardware is starting!"))

    return ld
