#!/usr/bin/env python3
import os
import subprocess
import time

# --- Terminal Styling ---
BLUE, GREEN, YELLOW, RED, BOLD, RESET = "\033[94m", "\033[92m", "\033[93m", "\033[91m", "\033[1m", "\033[0m"

def header(text):
    print(f"\n{BOLD}{BLUE}=== {text} ==={RESET}")

def run_cmd(cmd):
    try:
        # Prepend sourcing to every command to ensure ROS 2 is visible
        full_cmd = f"source /opt/ros/humble/setup.bash && source /open_agbot_ws/install/setup.bash && {cmd}"
        return subprocess.check_output(full_cmd, shell=True, executable="/bin/bash", stderr=subprocess.STDOUT).decode().strip()
    except subprocess.CalledProcessError as e:
        return None

header("SYSTEM & HARDWARE PROBE")

# 1. Serial Ports
for dev in ["/dev/ttyACM0", "/dev/ttyACM1"]:
    status = "[✓]" if os.path.exists(dev) else "[✗]"
    color = GREEN if os.path.exists(dev) else RED
    print(f"{color}{status} Device {dev}{RESET}")

# 2. Advanced Serial Probe
print(f"{YELLOW}[?] Probing ACM0 (GPS) Protocol...{RESET}")
probe_raw = subprocess.run(f"timeout 1 cat /dev/ttyACM0 | head -c 100", shell=True, capture_output=True)
if b'$G' in probe_raw.stdout:
    print(f"    {GREEN}Detected NMEA Sentences (Standard GPS).{RESET}")
elif len(probe_raw.stdout) > 0:
    print(f"    {BLUE}Detected Binary Stream (Likely UBX/RTK).{RESET}")
else:
    print(f"    {RED}No data on wire.{RESET}")

header("ROS 2 GRAPH HEALTH")

# 1. Node Check
nodes = run_cmd("ros2 node list") or ""
target_nodes = ["ublox_gps_node", "basekit_driver_node", "basekit_ui_node"]
for node in target_nodes:
    if f"/{node}" in nodes:
        print(f"{GREEN}[✓] Node /{node} is active{RESET}")
    else:
        print(f"{RED}[✗] Node /{node} is DOWN{RESET}")

# 2. Topic/Frequency Check
header("TOPIC DATA FLOW")
test_topics = {
    "/ublox_gps_node/fix": "GPS Position",
    "/v_batt": "Battery Voltage",
    "/motor_status": "Drive Motors"
}

for topic, label in test_topics.items():
    print(f"{YELLOW}[?] Testing {label}...{RESET}")
    # Check if topic exists first
    topic_info = run_cmd(f"ros2 topic info {topic}")
    if topic_info and "Publisher count: 0" not in topic_info:
        hz_output = run_cmd(f"timeout 3 ros2 topic hz {topic}")
        if hz_output and "average rate" in hz_output:
            rate = hz_output.split("average rate:")[1].split("\n")[0].strip()
            print(f"    {GREEN}[✓] {label}: {rate} Hz{RESET}")
        else:
            print(f"    {YELLOW}[!] {label}: Topic exists but NO DATA.{RESET}")
    else:
        print(f"    {RED}[✗] {label}: No publishers found.{RESET}")

header("DIAGNOSTIC COMPLETE")
