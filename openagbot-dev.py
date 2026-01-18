import os
import subprocess

def launch():
    ws_path = os.getcwd()
    
    # We explicitly source humble and the workspace install
    # and force the PYTHONPATH to look at ROS 2 directories
    setup_cmds = (
        "source /opt/ros/humble/setup.bash && "
        "mkdir -p /opt/lizard && "
        "cp /workspace/src/basekit_driver/scripts/espresso.py /opt/lizard/espresso.py && "
        "chmod +x /opt/lizard/espresso.py && "
        "colcon build --symlink-install && "
        "source /workspace/install/setup.bash && "
        "export PYTHONPATH=$PYTHONPATH:/opt/ros/humble/lib/python3.10/site-packages && "
        "ros2 launch basekit_launch basekit_launch.py"
    )

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

    print("--- 🚀 Launching with Path Protection ---")
    try:
        subprocess.run(docker_cmd)
    except KeyboardInterrupt:
        print("\n--- 🛑 Shutdown ---")

if __name__ == "__main__":
    launch()
