#!/usr/bin/env python3
import os, subprocess, time, curses, re

# Configuration
EXPECTED_NODES = ["/basekit_driver_node", "/ublox_gps_node", "/web_ui"]
TARGET_TOPICS = {
    "GPS Fix": "/gps/fix",
    "Battery": "/battery_state",
    "Odometry": "/odom",
    "Motor Cmds": "/cmd_vel"
}
PORTS = {"GPS": "/dev/ttyACM0", "MCU": "/dev/ttyACM1"}

def run_cmd(cmd):
    try:
        # Container-native pathing /workspace
        source_cmd = "source /opt/ros/humble/setup.bash && [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash"
        full_cmd = "{0}; {1}".format(source_cmd, cmd)
        return subprocess.check_output(full_cmd, shell=True, executable="/bin/bash", stderr=subprocess.DEVNULL).decode().strip()
    except:
        return ""

def draw(stdscr):
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1) 
    curses.init_pair(2, curses.COLOR_RED, -1)   
    curses.init_pair(3, curses.COLOR_CYAN, -1)  
    curses.init_pair(4, curses.COLOR_YELLOW, -1)
    
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(1000)
    
    while True:
        stdscr.clear()
        nodes_list = run_cmd("ros2 node list")
        topics_list = run_cmd("ros2 topic list")
        
        stdscr.addstr(0, 0, "🚀 AGBOT MISSION CONTROL V4.7", curses.color_pair(3) | curses.A_BOLD)
        stdscr.addstr(1, 0, "============================================================", curses.color_pair(3))

        stdscr.addstr(3, 0, "SERIAL PORTS", curses.A_UNDERLINE)
        for i, (label, port) in enumerate(PORTS.items()):
            exists = os.path.exists(port)
            status = "[OK]" if exists else "[LOST]"
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(4+i, 2, "{0:<4} {1:<14}: {2}".format(label, port, status), col)

        stdscr.addstr(7, 0, "ACTIVE NODES", curses.A_UNDERLINE)
        for i, node in enumerate(EXPECTED_NODES):
            exists = node in nodes_list
            status = "[ACTIVE]" if exists else "[OFFLINE]"
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(8+i, 2, "{0:<25}: {1}".format(node, status), col)

        stdscr.addstr(12, 0, "TOPIC STREAMS", curses.A_UNDERLINE)
        for i, (label, topic) in enumerate(TARGET_TOPICS.items()):
            exists = topic in topics_list
            status = "STREAMING" if exists else "WAITING..."
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(13+i, 2, "{0:<12}:".format(label), curses.A_DIM)
            stdscr.addstr(13+i, 20, status, col)

        stdscr.addstr(18, 0, "Press 'q' for VERBOSE POST-FLIGHT AUDIT", curses.color_pair(4))
        stdscr.refresh()
        if stdscr.getch() == ord('q'):
            break

if __name__ == "__main__":
    try:
        curses.wrapper(draw)
    except:
        pass

    print("\n" + "="*70)
    print("🔎 VERBOSE ROS 2 GRAPH AUDIT")
    print("="*70)

    # Dynamic Node Discovery
    nodes_raw = run_cmd("ros2 node list")
    found_nodes = [n for n in nodes_raw.split('\n') if n]
    
    for node in found_nodes:
        print("\n● {0}".format(node.upper()))
        node_info = run_cmd("ros2 node info {0}".format(node))
        
        # Regex to find Publishers
        pubs = re.findall(r'Publishers:(.*?)Service Servers:', node_info, re.S)
        if pubs:
            clean_pubs = [p.strip() for p in pubs[0].split('\n') if '/' in p]
            print("  ├─ Publishers: {0}".format(', '.join(clean_pubs[:4])))

    print("\n" + "="*70)
    print("📡 TOPIC CONTENT PREVIEW")
    print("="*70)
    for label, topic in TARGET_TOPICS.items():
        echo_data = run_cmd("timeout 0.5s ros2 topic echo {0} --once --no-arr".format(topic))
        if echo_data:
            clean_text = echo_data.replace('\n', ' | ')
            print("✅ {0:<10} : {1}...".format(label, clean_text[:100]))
        else:
            print("❌ {0:<10} : NO DATA".format(label))

    print("\n" + "="*40)
    print("📊 AGBOT POST-FLIGHT SUMMARY")
    print("="*40)
    
    final_nodes = run_cmd("ros2 node list")
    for node in EXPECTED_NODES:
        node_status = "✅ ONLINE" if node in final_nodes else "❌ OFFLINE"
        print("{0:<25} : {1}".format(node, node_status))
        
    print("="*40)
    print("Mission Control Session Ended.\n")
