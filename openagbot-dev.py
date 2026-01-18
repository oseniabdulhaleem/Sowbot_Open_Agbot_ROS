import os
import subprocess

def launch():
    # Get current workspace directory (should be /home/sam/open_agbot_ws)
    ws_path = os.getcwd()
    
    # Combined shell commands to run INSIDE the container:
    # 1. Source the system ROS 2 environment
    # 2. Build the workspace (using symlinks for live Python editing)
    # 3. Source the local workspace
    # 4. Launch the full AgBot stack
    setup_cmds = (
        "source /opt/ros/humble/setup.bash && "
        "colcon build --symlink-install && "
        "source install/setup.bash && "
        "ros2 launch basekit_launch basekit_launch.py"
    )

    # Docker execution configuration
    docker_cmd = [
        "docker", "run", "-it", "--rm",
        "--name", "basekit_dev",
        "--privileged",           # Required for GPIO/Serial access
        "--network", "host",      # Required for NiceGUI web access
        "-v", "/dev:/dev",        # Map hardware devices (GPS/MCU)
        "-v", f"{ws_path}:/workspace", # Map your code into the container
        
        # Inject Python paths directly to avoid ModuleNotFoundErrors
        "-e", "PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/workspace/install/basekit_ui/lib/python3.10/site-packages:/workspace/install/basekit_driver/lib/python3.10/site-packages",
        
        # Set home to tmp to avoid permission issues with config files
        "-e", "XDG_CACHE_HOME=/tmp", 
        "-e", "HOME=/tmp",
        
        "openagbot-basekit:latest",
        "bash", "-c", setup_cmds
    ]

    print("--- 🚀 Launching Open AgBot Dev Environment ---")
    print(f"Workspace detected at: {ws_path}")
    print("Point your browser to http://localhost:8080 once the UI node starts.")
    print("-------------------------------------------------------")
    
    try:
        # Run the docker command and stream output to terminal
        subprocess.run(docker_cmd)
    except KeyboardInterrupt:
        print("\n--- 🛑 Shutting down AgBot ---")

if __name__ == "__main__":
    launch()
