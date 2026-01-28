import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Capture Arguments (Passed from manage.py)
    sim = LaunchConfiguration('sim')
    gps_device = LaunchConfiguration('device')      # /dev/ttyACM0 from .env
    mcu_port = LaunchConfiguration('mcu_port')      # /dev/ttyACM1 from .env
    map_name = LaunchConfiguration('map_name')

    # 2. GPS Driver Node (ZED-F9P)
    # This node is critical to fix the "Invalid frame ID map" errors.
    ublox_gps_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='screen',
        parameters=[{
            'device': gps_device,
            'baudrate': 460800,
            'frame_id': 'gps',
            'use_sim_time': sim,
            'publish': {'nav': {'pose': True}}
        }]
    )

    # 3. Basekit Driver Node (MCU Interface)
    # Connects to ESP32 for battery, bumpers, and motors.
    base_driver_node = Node(
        package='basekit_driver',
        executable='basekit_driver_node',
        name='basekit_driver_node',
        output='screen',
        parameters=[{
            'use_sim_time': sim,
            'mcu_port': mcu_port,
            'gps_port': gps_device
        }]
    )

    # 4. Map Manager
    map_manager = Node(
        package="topological_navigation", 
        executable="map_manager2.py", 
        name="map_manager", 
        arguments=['/workspace/maps/test_map.yaml'],
        parameters=[{"tmap_file": "/workspace/maps/test_map.yaml", "use_sim_time": sim}]
    )

    # 5. Nav2 Controller Server
    # Handles the local costmap and path following.
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[{'use_sim_time': sim}]
    )

    # 6. Lifecycle Manager
    # Controls the state of the controller_server.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': sim,
            'autostart': True,
            'node_names': ['controller_server']
        }]
    )

    # 7. Topological Navigation
    topological_nav = Node(
        package='topological_navigation',
        executable='navigation2.py',
        name='topological_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': sim,
            'map_name': map_name
        }]
    )

    # 8. Web UI Node
    web_ui = Node(
        package='basekit_ui',
        executable='basekit_ui_node',
        name='basekit_ui',
        parameters=[{'use_sim_time': sim}]
    )

    return LaunchDescription([
        # Default Arguments
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('device', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('mcu_port', default_value='/dev/ttyACM1'),
        DeclareLaunchArgument('map_name', default_value='agbot_field'),
        
        # Nodes
        ublox_gps_node,
        base_driver_node,
        map_manager,
        controller_server,
        lifecycle_manager,
        topological_nav,
        web_ui
    ])
