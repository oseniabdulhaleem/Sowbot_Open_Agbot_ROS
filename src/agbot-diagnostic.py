#!/usr/bin/env python3
import os
import subprocess
import re

BLUE, GREEN, YELLOW, RED, BOLD, RESET = "\033[94m", "\033[92m", "\033[93m", "\033[91m", "\033[1m", "\033[0m"
SUCCESS, FAIL, WARN = f"{GREEN}[✓]{RESET}", f"{RED}[✗]{RESET}", f"{YELLOW}[❗]{RESET}"

def header(text):
    print(f"\n{BOLD}{BLUE}=== {text} ==={RESET}")

def run_cmd(cmd):
    try:
        full_cmd = f"source /opt/ros/humble/setup.bash && [ -f /open_agbot_ws/install/setup.bash ] && source /open_agbot_ws/install/setup.bash; {cmd}"
        return subprocess.check_output(full_cmd, shell=True, executable="/bin/bash", stderr=subprocess.DEVNULL).decode().strip()
    except:
        return None

header("SERIAL HARDWARE AVAILABILITY")
for dev in ["/dev/ttyACM0", "/dev/ttyACM1"]:
    if os.path.exists(dev):
        # Check if another process is already using the port
        lsof = subprocess.getoutput(f"lsof {dev}")
        status = SUCCESS if not lsof else WARN
        print(f"{status} {dev}: Available")
        if lsof:
            print(f"    {RED}↳ PORT BUSY:{RESET} Used by PID {lsof.splitlines()[-1].split()[1]}")
    else:
        print(f"{FAIL} {dev}: NOT FOUND")

header("ROS 2 GRAPH: TOPIC & NODE HEALTH")
nodes = run_cmd("ros2 node list") or ""
print(f"{BOLD}Active Nodes:{RESET}\n{nodes if nodes else 'None'}")

topics = ["/fix", "/v_batt", "/motor_status"]
for t in topics:
    info = run_cmd(f"ros2 topic info {t}")
    if info and "Publisher count: 0" not in info:
        hz = run_cmd(f"timeout 1s ros2 topic hz {t}")
        rate = re.findall(r"average rate: ([\d.]+)", hz) if hz else None
        print(f"{SUCCESS} {t}: {GREEN}{rate[0] + ' Hz' if rate else 'Streaming'}{RESET}")
    else:
        print(f"{FAIL} {t}: No Publisher found.")

header("DIAGNOSTIC COMPLETE")
