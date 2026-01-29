import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

# --- VERSIONING ---
VERSION = "5.4-STABLE"

def generate_launch_description():
    ld = LaunchDescription()

    # 1. Environment & Config Logic
    mode = os.environ.get('MODE', 'simulation').lower()
    gps_port = os.environ.get('GPS_PORT', '/dev/ttyACM0')
    mcu_port = os.environ.get('MCU_PORT', '/dev/ttyACM1')
    is_sim = (mode == 'simulation')

    # Version Banner
    ld.add_action(LogInfo(msg="======================================================"))
    ld.add_action(LogInfo(msg=f"🚀 AGBOT MASTER LAUNCH V{VERSION}"))
    ld.add_action(LogInfo(msg=f"🎮 MODE: {mode.upper()} | NAVIGATION STACK ACTIVE"))
    ld.add_action(LogInfo(msg="======================================================"))

    # 2. UI & Communication Bridge
    ld.add_action(Node(
        package='rosbridge_server', executable='rosbridge_websocket',
        name='rosbridge_websocket', output='screen',
        parameters=[{'port': 9090, 'address': '0.0.0.0'}]
    ))

    ld.add_action(Node(
        package='basekit_ui', executable='basekit_ui_node',
        name='web_ui', output='screen',
        parameters=[{'use_sim_time': is_sim}]
    ))

    # 3. Restored Navigation Stack (Matches Audit)
    # Lifecycle Manager coordinates the state of controller and costmaps
    ld.add_action(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': is_sim,
            'autostart': True,
            'node_names': ['controller_server', 'local_costmap', 'costmap']
        }]
    ))

    ld.add_action(Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[{'use_sim_time': is_sim}]
    ))

    ld.add_action(Node(
        package='nav2_costmap_2d', executable='nav2_costmap_2d',
        name='local_costmap', namespace='local_costmap',
        output='screen', parameters=[{'use_sim_time': is_sim}]
    ))

    # 4. Topological Navigation (From Commit a7232f5)
    ld.add_action(Node(
        package="topological_navigation", executable="map_manager2.py",
        name="map_manager", arguments=['/workspace/maps/test_map.yaml'],
        parameters=[{"tmap_file": "/workspace/maps/test_map.yaml"}]
    ))

    ld.add_action(Node(
        package='topological_navigation', executable='navigation2.py',
        name='topological_navigation', output='screen',
        parameters=[{'use_sim_time': is_sim, 'map_name': 'test_map'}]
    ))

    # 5. Conditional Hardware/Mock Drivers
    if not is_sim:
        ld.add_action(Node(
            package='ublox_gps', executable='ublox_gps_node', name='ublox_gps_node',
            parameters=[{'device': gps_port}], respawn=True
        ))
        ld.add_action(Node(
            package='basekit_driver', executable='basekit_driver_node', 
            name='basekit_driver_node', parameters=[{'port': mcu_port}], respawn=True
        ))
    else:
        # Launching driver in virtual mode to satisfy 'Battery' and 'Bumper' publishers
        ld.add_action(Node(
            package='basekit_driver', executable='basekit_driver_node',
            name='basekit_driver_node', parameters=[{'use_sim_time': True, 'port': 'virtual'}]
        ))

    return ld
