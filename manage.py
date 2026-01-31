#!/usr/bin/env python3
import subprocess
import os
import sys
import time

# Image and container naming configuration
IMAGE_NAME = "openagbot:dev"
CONTAINER_NAME = "open_ag_runtime"

def run_build(full=False):
    """
    Cleans local workspace, builds the Docker image, and syncs
    build artifacts back to the host for local development.
    """
    # 1. CLEAN: Remove artifacts first so host checks are accurate
    print("Cleaning host artifacts...")
    subprocess.run(['sudo', 'rm', '-rf', 'build/', 'install/', 'log/'], check=False)
    
    # 2. IMAGE BUILD: Update the base Docker image
    print("Building Docker Image...")
    build_cmd = ["docker", "build", "-t", IMAGE_NAME, "-f", "docker/Dockerfile", "."]
    if full: 
        build_cmd.insert(2, "--no-cache")
    subprocess.run(build_cmd, check=True)

    # 3. SYNC: Compile code and move artifacts to host
    print("Syncing build artifacts to host...")
    user_info = f"{os.getuid()}:{os.getgid()}"
    
    # Determine what to source. Since we just cleaned, this will usually be the Humble base.
    if not os.path.exists("install/setup.bash"):
        setup_cmd = "source /opt/ros/humble/setup.bash"
    else:
        setup_cmd = "source /opt/ros/humble/setup.bash && source install/setup.bash"

       
    
    # We use --entrypoint /bin/bash to STOP the image from 
    # trying to source the missing install folder on startup.
    cmd_sync = [
        "docker", "run", "--rm", 
        "--user", user_info,
        "--entrypoint", "/bin/bash", # <--- ADD THIS LINE
        "-v", f"{os.getcwd()}:/workspace", 
        "-w", "/workspace",
        IMAGE_NAME, "-c",            # <--- CHANGE "bash" TO "-c"
        f"{setup_cmd} && colcon build --symlink-install --executor sequential"
    ]
    
    
    
    
    
    
    
    
    try:
        subprocess.run(cmd_sync, check=True)
        print("Build and Sync Complete.")
    except subprocess.CalledProcessError as e:
        print(f"Build failed. If the error is 'No such file', check the Dockerfile ENTRYPOINT.")
        sys.exit(1)






def run_runtime(extra_args):
    """
    Handles hardware port preparation and launching the ROS 2 environment.
    """
    if os.path.exists('fixusb.py'):
        subprocess.run(['python3', 'fixusb.py'], check=True)

    ports = {}
    if os.path.exists('.env'):
        with open('.env', 'r') as f:
            for line in f:
                if '=' in line and not line.startswith('#'):
                    key, value = line.strip().split('=', 1)
                    ports[key.strip()] = value.strip()
    
    gps_p = ports.get('GPS_PORT', 'virtual')
    mcu_p = ports.get('MCU_PORT', 'virtual')
    is_virtual = (gps_p == 'virtual' and mcu_p == 'virtual')

    if not is_virtual:
        print(f"Configuring hardware: GPS={gps_p}, MCU={mcu_p}")
        subprocess.run(['sudo', 'chmod', '666', gps_p], check=False)
        subprocess.run(['sudo', 'chmod', '666', mcu_p], check=False)
        os.system(f"stty -F {mcu_p} 115200 && (echo 's' > {mcu_p} &)")

    subprocess.run(["docker", "rm", "-f", CONTAINER_NAME], capture_output=True)
    
    sim_flag = "sim:=true" if is_virtual else "sim:=false"
    
    # Check for local install again for the runtime phase
    if not os.path.exists("install/setup.bash"):
        setup_cmd = "source /opt/ros/humble/setup.bash"
    else:
        setup_cmd = f"source /opt/ros/humble/setup.bash && source install/setup.bash"

    print(f"Launching Runtime | GPS: {gps_p} | MCU: {mcu_p}")
    cmd = [
        "docker", "run", "-it", "--rm", "--name", CONTAINER_NAME,
        "--entrypoint", "/bin/bash", # Consistent override to prevent startup crashes
        "--net=host", "--privileged", "--env-file", ".env",
        "-v", "/dev:/dev", 
        "-v", f"{os.getcwd()}:/workspace",
        "-w", "/workspace",
        IMAGE_NAME, "-c",
        f"{setup_cmd} && "
        f"ros2 launch basekit_launch basekit_launch.py {sim_flag} "
        f"gps_port:={gps_p} mcu_port:={mcu_p} {' '.join(extra_args)}"
    ]
    subprocess.run(cmd)

if __name__ == "__main__":
    build_triggers = ["build", "full-build"]
    needs_build = not os.path.exists('install/setup.bash') or any(arg in sys.argv for arg in build_triggers)
    
    if needs_build:
        run_build("full-build" in sys.argv)
    
    if not any(arg in build_triggers for arg in sys.argv):
        launch_args = [arg for arg in sys.argv[1:] if arg != "up"]
        run_runtime(launch_args)
