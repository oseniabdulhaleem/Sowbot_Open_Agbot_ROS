#!/usr/bin/env python3
import subprocess
import os
import sys
import time

# Configuration
IMAGE_NAME = "openagbot:dev"
CONTAINER_NAME = "open_agbot"

def run_build():
    """
    Cleans up old container artifacts and builds the image fresh.
    """
    print("🧹 Removing old container cruft...")
    subprocess.run(["docker", "rm", "-f", CONTAINER_NAME], capture_output=True)
    subprocess.run(["docker", "system", "prune", "-f"], capture_output=True)
    
    print(f"🛠️ Building fresh image: {IMAGE_NAME}...")
    # --no-cache ensures we don't reuse broken hybrid layers
    cmd = [
        "docker", "build", "--no-cache", 
        "-t", IMAGE_NAME, 
        "-f", "docker/Dockerfile", "."
    ]
    result = subprocess.run(cmd)
    if result.returncode == 0:
        print("✅ Build successful.")
    else:
        print("❌ Build failed.")
    sys.exit(result.returncode)

def run_runtime(extra_args):
    """
    Final Stable Runtime: Hardware discovery and ROS 2 Launch.
    """
    # 1. Hardware Discovery
    if os.path.exists('fixusb.py'):
        print("🔍 Running fixusb.py...")
        subprocess.run(['python3', 'fixusb.py'], check=True)

    # 2. Load Environment
    ports = {}
    if os.path.exists('.env'):
        with open('.env', 'r') as f:
            for line in f:
                if '=' in line and not line.startswith('#'):
                    k, v = line.strip().split('=', 1)
                    ports[k.strip()] = v.strip()
    
    gps_p = ports.get('GPS_PORT', 'virtual')
    mcu_p = ports.get('MCU_PORT', 'virtual')
    is_virtual = (gps_p == 'virtual' and mcu_p == 'virtual')

    # 3. Cleanup & Prep
    print("🧹 Cleaning up previous container...")
    subprocess.run(["docker", "rm", "-f", CONTAINER_NAME], capture_output=True)
    
    if not is_virtual:
        print(f"✅ Hardware | GPS: {gps_p} | MCU: {mcu_p}")
        subprocess.run(['sudo', 'chmod', '666', gps_p, mcu_p], check=False)
        
        # Non-blocking MCU Wakeup (Prevents terminal hang)
        print(f"🔋 Waking up MCU on {mcu_p}...")
        os.system(f"stty -F {mcu_p} 115200 && (echo 's' > {mcu_p} &)")
        time.sleep(0.2)
    else:
        print("🎮 Mode: Simulation")

    # 4. ROS 2 Launch
    try:
        # Filter out "build" or "up" from extra_args to prevent ROS malformed arg error
        launch_args = [arg for arg in extra_args if ":=" in arg]
        sim_flag = "sim:=true" if is_virtual else "sim:=false"
        
        print(f"🚀 Launching {CONTAINER_NAME}...")
        cmd = [
            "docker", "run", "-it", "--rm", "--name", CONTAINER_NAME,
            "--net=host", "--privileged", "--env-file", ".env",
            "-v", "/dev:/dev", 
            "-v", f"{os.getcwd()}:/workspace", "-w", "/workspace",
            IMAGE_NAME, "bash", "-c",
            f"source /opt/ros/humble/setup.bash && source install/setup.bash && "
            f"ros2 launch basekit_launch basekit_launch.py {sim_flag} "
            f"device:={gps_p} mcu_port:={mcu_p} {' '.join(launch_args)}"
        ]
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\n🛑 Shutdown.")

if __name__ == "__main__":
    raw_args = sys.argv[1:]
    
    if "build" in raw_args:
        run_build()
    
    # Strip "up" if present
    processed_args = [a for a in raw_args if a != "up"]
    run_runtime(processed_args)
