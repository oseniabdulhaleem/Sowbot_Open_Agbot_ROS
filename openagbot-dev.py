import os
import subprocess

def launch():
    ws_path = os.getcwd()
    
    # We force the container to use the system Python 3.10 site-packages
    # where ROS 2 Humble is natively installed.
    python_path = (
        "/opt/ros/humble/lib/python3.10/site-packages:"
        "/usr/lib/python3/dist-packages:"
        "/workspace/install/basekit_ui/lib/python3.10/site-packages:"
        "/workspace/install/basekit_driver/lib/python3.10/site-packages"
    )

    setup_cmds = (
        "source /opt/ros/humble/setup.bash && "
        "colcon build --symlink-install && "
        "source install/setup.bash && "
        "ros2 launch basekit_launch basekit_launch.py"
    )

    docker_cmd = [
        "docker", "run", "-it", "--rm",
        "--privileged",
        "--network", "host",
        "-v", "/dev:/dev",
        "-v", f"{ws_path}:/workspace",
        "-e", f"PYTHONPATH={python_path}",
        "-e", "PYTHONUNBUFFERED=1",
        "openagbot-basekit:latest",
        "bash", "-c", setup_cmds
    ]

    try:
        subprocess.run(docker_cmd)
    except KeyboardInterrupt:
        print("\nStopping...")

if __name__ == "__main__":
    launch()
