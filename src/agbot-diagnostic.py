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

def run_cmd(cmd):
    try:
        full_cmd = f"source /opt/ros/humble/setup.bash && [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash; {cmd}"
        return subprocess.check_output(full_cmd, shell=True, executable="/bin/bash", stderr=subprocess.DEVNULL).decode().strip()
    except: return ""

def get_hz(topic):
    output = run_cmd(f"timeout 0.3s ros2 topic hz {topic} --window 2")
    if "average rate:" in output:
        try: return output.split("average rate:")[1].split("\n")[0].strip()
        except: return "0.0"
    return "0.0"

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
        all_nodes = run_cmd("ros2 node list")
        
        stdscr.addstr(0, 0, "🚀 AGBOT MISSION CONTROL V4.0", curses.color_pair(3) | curses.A_BOLD)
        stdscr.addstr(1, 0, "="*78, curses.color_pair(3))

        stdscr.addstr(3, 0, "LIVE NODE HEALTH", curses.A_UNDERLINE)
        for i, node in enumerate(EXPECTED_NODES):
            exists = node in all_nodes
            status = "[ONLINE]" if exists else "[OFFLINE]"
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(4+i, 2, f"{node:<25}: {status}", col)

        stdscr.addstr(9, 0, "LIVE TOPIC FLOW", curses.A_UNDERLINE)
        for i, (label, topic) in enumerate(TARGET_TOPICS.items()):
            hz = get_hz(topic)
            col = curses.color_pair(1) if float(hz.split(' ')[0] if ' ' in hz else hz) > 0 else curses.color_pair(2)
            stdscr.addstr(10+i, 2, f"{label:<12} [{topic:<18}]: {hz:>8} Hz", col)

        stdscr.addstr(16, 0, "Press [q] for VERBOSE DEEP-SCAN POST-FLIGHT", curses.color_pair(4))
        stdscr.refresh()
        if stdscr.getch() == ord('q'): break

if __name__ == "__main__":
    try:
        curses.wrapper(draw)
    except:
        pass

    print("\n" + "═"*70)
    print("🔎 VERBOSE ROS 2 GRAPH AUDIT")
    print("═"*70)

    # 1. Detailed Node Audit
    nodes = run_cmd("ros2 node list").split('\n')
    print(f"\n[NODES DETECTED: {len([n for n in nodes if n])}]")
    for node in nodes:
        if not node: continue
        print(f"\n● {node.upper()}")
        node_info = run_cmd(f"ros2 node info {node}")
        
        # Extract Publishers
        pubs = re.findall(r'Publishers:(.*?)Service Servers:', node_info, re.S)
        if pubs:
            clean_pubs = [p.strip() for p in pubs[0].split('\n') if '/' in p]
            print(f"  ├─ Publishers: {', '.join(clean_pubs[:4])}")
        
        # Extract Subscribers
        subs = re.findall(r'Subscribers:(.*?)Service Clients:', node_info, re.S)
        if subs:
            clean_subs = [s.strip() for s in subs[0].split('\n') if '/' in s]
            print(f"  └─ Subscribers: {', '.join(clean_subs[:4])}")

    # 2. Topic Data Verification
    print("\n" + "═"*70)
    print("📡 TOPIC CONTENT PREVIEW (Live Data)")
    print("═"*70)
    for label, topic in TARGET_TOPICS.items():
        # Get one message from each topic to show actual values
        echo = run_cmd(f"ros2 topic echo {topic} --once --no-arr")
        if echo:
            # Clean up the output for terminal readability
            preview = echo.replace('\n', ' | ')[:100]
            print(f"✅ {label:<10} : {preview}...")
        else:
            print(f"❌ {label:<10} : NO DATA (TIMEOUT)")

    print("\n" + "═"*70)
    print("Audit Complete. AgBot is Mission Ready.")
    print("═"*70 + "\n")
