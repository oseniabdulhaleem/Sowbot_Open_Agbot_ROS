#!/usr/bin/env python3
import subprocess
import sys
import os

def run_cmd(cmd):
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"[✘] Command failed: {e}")
        sys.exit(1)

def build():
    print("--- Starting Container Build ---")
    run_cmd(["docker", "build", "-t", "openagbot:dev", "-f", "docker/Dockerfile", "."])

def up():
    print("--- Starting Environment ---")
    # Trigger USB port identification before container start
    if os.path.exists("fixusb.py"):
        run_cmd(["python3", "fixusb.py"])
    
    # Target the verified file location
    compose_path = "docker/docker-compose.yml"
    
    if not os.path.exists(compose_path):
        print(f"[✘] Error: {compose_path} not found.")
        sys.exit(1)
            
    run_cmd(["docker-compose", "-f", compose_path, "up", "-d"])

def down():
    run_cmd(["docker-compose", "-f", "docker/docker-compose.yml", "down"])

def shell():
    run_cmd(["docker", "exec", "-it", "open_ag_runtime", "bash"])

def logs():
    run_cmd(["docker-compose", "-f", "docker/docker-compose.yml", "logs", "-f"])

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 manage.py [build|up|down|shell|logs]")
        sys.exit(1)
    
    cmd = sys.argv[1]
    commands = {
        "build": build,
        "up": up,
        "down": down,
        "shell": shell,
        "logs": logs
    }
    
    if cmd in commands:
        commands[cmd]()
    else:
        print(f"Unknown command: {cmd}")
        sys.exit(1)
