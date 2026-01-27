#!/usr/bin/env python3
import subprocess
import os
import sys
import signal
import time

# --- Path Configuration ---
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DOCKER_DIR = os.path.join(BASE_DIR, "docker")
COMPOSE_FILE = os.path.join(DOCKER_DIR, "docker-compose.yml")
DOCKERFILE = os.path.join(DOCKER_DIR, "Dockerfile")
ENV_FILE = os.path.join(BASE_DIR, ".env")

def ensure_service(name, docker_cmd):
    """Generic check to see if a support container is running."""
    print(f"Action: Checking {name} status...")
    check_cmd = f"docker ps --filter 'name={name}' --filter 'status=running' -q"
    is_running = subprocess.run(check_cmd, shell=True, capture_output=True, text=True).stdout.strip()
    
    if not is_running:
        print(f"Action: Launching {name}...")
        try:
            subprocess.run(docker_cmd, shell=True, check=True)
            time.sleep(2)
        except subprocess.CalledProcessError:
            print(f"Failed to start {name}")
    else:
        print(f"Status: {name} is active.")

def ensure_infrastructure():
    # 1. MongoDB
    ensure_service("mongodb", f"docker run -d --name mongodb -p 27017:27017 mongo:6.0")
    
    # 2. Portainer (Community Edition)
    portainer_cmd = (
        "docker run -d -p 8000:8000 -p 9443:9443 --name portainer "
        "--restart=always -v /var/run/docker.sock:/var/run/docker.sock "
        "-v portainer_data:/data portainer/portainer-ce:latest"
    )
    ensure_service("portainer", portainer_cmd)

def run_build():
    print("--- Starting Build Phase ---")
    # Ensure Dockerfile has Nav2 dependencies


    
    subprocess.run(['sudo', 'rm', '-rf', 'build/', 'install/', 'log/'], check=False)
    
    print("Step 1: Building Docker Image (Adding Nav2)...")
    subprocess.run(["docker", "build", "-t", "openagbot:dev", "-f", DOCKERFILE, BASE_DIR], check=True)

    print("Step 2: Compiling ROS 2 Workspace...")
    user_info = f"{os.getuid()}:{os.getgid()}"
    cmd_sync = [
        "docker", "run", "--rm", "--user", user_info,
        "-v", f"{BASE_DIR}:/workspace", "-w", "/workspace",
        "openagbot:dev", "bash", "-c", 
        "source /opt/ros/humble/setup.bash && colcon build --symlink-install --executor sequential --parallel-workers 1"
    ]
    subprocess.run(cmd_sync, check=True)

def run_runtime(extra_args):
    if os.path.exists('fixusb.py'):
        subprocess.run(['python3', 'fixusb.py'], check=True)
    
    ensure_infrastructure()
    
    ports = {"GPS_PORT": "virtual", "MCU_PORT": "virtual"}
    if os.path.exists(ENV_FILE):
        with open(ENV_FILE, 'r') as f:
            for line in f:
                if '=' in line and not line.startswith('#'):
                    k, v = line.strip().split('=', 1)
                    ports[k.strip()] = v.strip()
    
    is_virtual = (ports['GPS_PORT'] == 'virtual' and ports['MCU_PORT'] == 'virtual')
    subprocess.run(["docker", "rm", "-f", "open_agbot"], capture_output=True)

    sim_proc = None
    if is_virtual:
        print("Mode: Simulation (Ghost)")
        if os.path.exists('sim_lizard.sh'):
            sim_proc = subprocess.Popen(['bash', 'sim_lizard.sh'], stdout=subprocess.DEVNULL, preexec_fn=os.setsid)

    try:
        sim_flag = "sim:=true" if is_virtual else "sim:=false"
        print(f"Launching open_agbot | GPS: {ports['GPS_PORT']} | MCU: {ports['MCU_PORT']}")
        
        cmd = [
            "docker", "run", "-it", "--rm", "--name", "open_agbot",
            "--net=host", "--privileged", "--env-file", ENV_FILE,
            "-v", "/dev:/dev", "-v", f"{BASE_DIR}:/workspace", "-w", "/workspace",
            "openagbot:dev", "bash", "-c",
            f"source /opt/ros/humble/setup.bash && source install/setup.bash && "
            f"ros2 launch basekit_launch basekit_launch.py {sim_flag} "
            f"gps_port:={ports['GPS_PORT']} mcu_port:={ports['MCU_PORT']} {' '.join(extra_args)}"
        ]
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        if sim_proc:
            os.killpg(os.getpgid(sim_proc.pid), signal.SIGTERM)

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "build":
        run_build()
    else:
        if not os.path.exists('install/setup.bash'):
            run_build()
        run_runtime(sys.argv[1:])
