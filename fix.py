import os
import subprocess

def run_agbot():
    ws_path = os.path.expanduser("~/open_agbot_ws")
    host_scripts_dir = os.path.join(ws_path, "src/basekit_driver/scripts")
    host_espresso = os.path.join(host_scripts_dir, "espresso.py")
    
    print("--- 1. Cleaning Workspace Artifacts (as Root) ---")
    # Using sudo rm -rf to bypass the PermissionError
    for folder in ['build', 'install', 'log']:
        path = os.path.join(ws_path, folder)
        if os.path.exists(path):
            subprocess.run(["sudo", "rm", "-rf", path])
            print(f"Deleted {folder}/")

    print("\n--- 2. Ensuring Firmware Script Exists ---")
    if not os.path.exists(host_espresso):
        os.makedirs(host_scripts_dir, exist_ok=True)
        # Extract from image
        subprocess.run(f"docker run --rm openagbot-basekit:latest cat /root/.lizard/espresso.py > {host_espresso}", shell=True)
        os.chmod(host_espresso, 0o755)
        print("Rescued espresso.py to host.")

    # 3. Define the Startup Sequence
    setup_cmds = (
        "mkdir -p /opt/lizard && "
        "cp /workspace/src/basekit_driver/scripts/espresso.py /opt/lizard/espresso.py && "
        "chmod +x /opt/lizard/espresso.py && "
        "colcon build && "
        "source install/setup.bash && "
        "ros2 launch basekit_launch basekit_launch.py"
    )

    # 4. Construct Docker Command
    docker_cmd = [
        "docker", "run", "-it", "--rm",
        "--name", "basekit_dev",
        "--privileged", "--network", "host",
        "-v", "/dev:/dev",
        "-v", f"{ws_path}:/workspace",
        "-e", "XDG_CACHE_HOME=/tmp", 
        "-e", "HOME=/tmp",
        "openagbot-basekit:latest",
        "bash", "-c", setup_cmds
    ]

    print("\n--- 3. Starting Open AgBot Stack ---")
    try:
        subprocess.run(docker_cmd)
    except KeyboardInterrupt:
        print("\nShutting down...")

if __name__ == "__main__":
    run_agbot()
