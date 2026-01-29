#!/usr/bin/env python3
import os, subprocess, time, curses, re

# --- VERSIONING ---
VERSION = "5.0-STABLE"

def get_env_config():
    """Reads hardware ports and mode dynamically from .env."""
    config = {"GPS": "NOT_SET", "MCU": "NOT_SET", "MODE": "unknown"}
    env_path = "/workspace/.env"
    if os.path.exists(env_path):
        with open(env_path, "r") as f:
            content = f.read()
            # Dynamic regex to pull fixusb.py results
            gps = re.search(r"GPS_PORT=(.*)", content)
            mcu = re.search(r"MCU_PORT=(.*)", content)
            mode = re.search(r"MODE=(.*)", content)
            if gps: config["GPS"] = gps.group(1).strip()
            if mcu: config["MCU"] = mcu.group(1).strip()
            if mode: config["MODE"] = mode.group(1).strip()
    return config

# --- CONFIGURATION ---
EXPECTED_NODES = ["/basekit_driver_node", "/ublox_gps_node", "/web_ui", "/rosbridge_websocket"]
TARGET_TOPICS = {
    "GPS Fix": "/gps/fix",
    "Battery": "/battery_state",
    "Odometry": "/odom",
    "Motor Cmds": "/cmd_vel"
}

def run_cmd(cmd):
    """Safe ROS2 command execution via shell sourcing."""
    try:
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
    
    cfg = get_env_config()

    while True:
        stdscr.clear()
        nodes_list = run_cmd("ros2 node list")
        topics_list = run_cmd("ros2 topic list")
        
        stdscr.addstr(0, 0, "🚀 AGBOT MISSION CONTROL V{0}".format(VERSION), curses.color_pair(3) | curses.A_BOLD)
        stdscr.addstr(1, 0, "Current Mode: {0}".format(cfg["MODE"].upper()), curses.color_pair(4))
        stdscr.addstr(2, 0, "="*65, curses.color_pair(3))

        # Hardware Section (Dynamic from .env)
        stdscr.addstr(4, 0, "HARDWARE (via fixusb.py)", curses.A_UNDERLINE)
        hw_labels = [("GPS", cfg["GPS"]), ("MCU", cfg["MCU"])]
        for i, (label, port) in enumerate(hw_labels):
            exists = os.path.exists(port) if port != "NOT_SET" else False
            status = "[ONLINE]" if exists else "[GHOST/SIM]"
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(5+i, 2, "{0:<4} {1:<18}: {2}".format(label, port, status), col)

        # Nodes Section
        stdscr.addstr(8, 0, "ACTIVE NODES", curses.A_UNDERLINE)
        for i, node in enumerate(EXPECTED_NODES):
            exists = node in nodes_list
            status = "[ACTIVE]" if exists else "[OFFLINE]"
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(9+i, 2, "{0:<28}: {1}".format(node, status), col)

        # Topic Streams
        stdscr.addstr(14, 0, "TOPIC STREAMS", curses.A_UNDERLINE)
        for i, (label, topic) in enumerate(TARGET_TOPICS.items()):
            exists = topic in topics_list
            status = "STREAMING" if exists else "WAITING..."
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(15+i, 2, "{0:<14}:".format(label), curses.A_DIM)
            stdscr.addstr(15+i, 20, status, col)

        stdscr.addstr(21, 0, "Press 'q' for VERBOSE POST-FLIGHT AUDIT", curses.color_pair(4))
        stdscr.refresh()
        if stdscr.getch() == ord('q'):
            break

if __name__ == "__main__":
    try:
        curses.wrapper(draw)
    except KeyboardInterrupt:
        pass

    # --- VERBOSE POST-FLIGHT AUDIT ---
    print("\n" + "═"*75)
    print("🔎 VERBOSE ROS 2 GRAPH AUDIT (V{0})".format(VERSION))
    print("═"*75)

    nodes_raw = run_cmd("ros2 node list")
    found_nodes = [n for n in nodes_raw.split('\n') if n]
    
    for node in found_nodes:
        print("\n● {0}".format(node.upper()))
        node_info = run_cmd("ros2 node info {0}".format(node))
        # Regex extraction of publishers
        pubs = re.findall(r'Publishers:(.*?)Service Servers:', node_info, re.S)
        if pubs:
            clean_pubs = [p.strip() for p in pubs[0].split('\n') if '/' in p]
            print("  ├─ Publishers: {0}".format(', '.join(clean_pubs[:4])))

    print("\n" + "═"*75)
    print("📡 TOPIC CONTENT PREVIEW")
    print("═"*75)
    for label, topic in TARGET_TOPICS.items():
        echo_data = run_cmd("timeout 0.5s ros2 topic echo {0} --once --no-arr".format(topic))
        if echo_data:
            clean_text = echo_data.replace('\n', ' | ')
            print("✅ {0:<12} : {1}...".format(label, clean_text[:90]))
        else:
            print("❌ {0:<12} : NO DATA".format(label))

    print("\n" + "="*45)
    print("📊 AGBOT POST-FLIGHT SUMMARY")
    print("="*45)
    
    final_nodes = run_cmd("ros2 node list")
    for node in EXPECTED_NODES:
        node_status = "✅ ONLINE" if node in final_nodes else "❌ OFFLINE"
        print("{0:<28} : {1}".format(node, node_status))
        
    print("="*45)
    print("Audit Complete.\n")
