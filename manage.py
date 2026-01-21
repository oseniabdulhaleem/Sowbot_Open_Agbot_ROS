#!/usr/bin/env python3
import argparse
import subprocess
import os
import sys

CONTAINER_WS = "/open_agbot_ws"
IMAGE_TAG = "openagbot:dev"

# Colors
YEL, GRN, RED, RST = "\033[93m", "\033[92m", "\033[91m", "\033[0m"

def log(msg, col=YEL): print(f"{col}[manage.py]{RST} {msg}")

def launch(mode="fast"):
    # 1. Sync hardware via fixusb
    subprocess.run(["python3", "fixusb.py"], check=True)
    if os.path.exists(".env"):
        with open(".env", "r") as f:
            for line in f:
                if "=" in line:
                    k, v = line.strip().split("=", 1)
                    os.environ[k] = v

    # 2. Extract detected ports from environment
    gps_port = os.environ.get('GPS_PORT')
    mcu_port = os.environ.get('MCU_PORT')
    
    if not gps_port or not mcu_port:
        log("❌ Hardware not found! Check connections.", RED)
        sys.exit(1)

    # 3. Cleanup locks and permissions on host
    subprocess.run(f"sudo fuser -k {gps_port} {mcu_port} || true", shell=True, capture_output=True)
    subprocess.run(f"sudo chmod 666 {gps_port} {mcu_port} || true", shell=True)
    subprocess.run("sudo pkill -9 -f _node || true", shell=True)

    # 4. ROS Commands
    param_path = f"{CONTAINER_WS}/install/basekit_driver/share/basekit_driver/config/basekit_driver.yaml"
    
    # We pass the ports explicitly to the launch file
    launch_cmd = (
        f"ros2 launch basekit_launch master.launch.py "
        f"gps_port:={gps_port} "
        f"mcu_port:={mcu_port} "
        f"device:={gps_port} "
        f"basekit_params_file:={param_path}"
    )
    
    setup_ros = "source /opt/ros/humble/setup.bash"
    
    if mode == "build":
        log(f"🛠️  Building & Launching (GPS: {gps_port}, MCU: {mcu_port})...", YEL)
        # Clear specific package artifacts and rebuild
        inner_cmd = f"{setup_ros} && rm -rf build/basekit_driver install/basekit_driver && colcon build --symlink-install --packages-select basekit_driver && source install/setup.bash && {launch_cmd}"
    else:
        log(f"🚀 Fast Launching (GPS: {gps_port}, MCU: {mcu_port})...", GRN)
        inner_cmd = f"{setup_ros} && source install/setup.bash && {launch_cmd}"

    # Docker Command
    docker_cmd = [
        "docker", "run", "-it", "--rm", "--name", "openagbot_app", "--privileged", "--network", "host",
        "-v", "/dev:/dev", "-v", f"{os.getcwd()}:{CONTAINER_WS}",
        "-e", "PYTHONUNBUFFERED=1", IMAGE_TAG, "bash", "-c", inner_cmd
    ]
    
    try:
        subprocess.run(docker_cmd)
    except KeyboardInterrupt:
        log("🛑 Stopping container.", YEL)

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument('cmd', nargs='?', choices=['run', 'build', 'clean'], default='run')
    args = p.parse_args()
    
    if args.cmd == 'clean':
        subprocess.run(["sudo", "rm", "-rf", "build", "install", "log"])
        log("Workspace cleaned.", GRN)
    else:
        launch(mode="build" if args.cmd == "build" else "fast")
