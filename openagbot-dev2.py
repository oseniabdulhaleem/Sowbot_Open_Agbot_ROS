import os
import subprocess
import sys
import shutil

class OpenAgBotDev:
    def __init__(self):
        self.workspace_root = os.path.dirname(os.path.abspath(__file__))
        self.compose_path = os.path.join(self.workspace_root, "docker/docker-compose.yml")
        self.docker_home = os.path.join(self.workspace_root, ".docker_home")
        
    def prepare_env(self):
        """Prepare host environment and directories."""
        print("--- 1. Preparing Environment ---")
        
        # Ensure persistent docker home exists for logs
        if not os.path.exists(self.docker_home):
            os.makedirs(self.docker_home)
            print(f"Created {self.docker_home}")

        # Map current user IDs for Docker
        self.env = os.environ.copy()
        self.env["UID"] = str(os.getuid())
        self.env["GID"] = str(os.getgid())
        
        # Fix potential X11 display issues for GUIs
        if "DISPLAY" not in self.env:
            self.env["DISPLAY"] = ":0"
            
        print(f"User IDs mapped: UID={self.env['UID']}, GID={self.env['GID']}")

    def manage_docker(self):
        """Ensure the container is running."""
        print("\n--- 2. Checking Docker Container ---")
        try:
            # up -d ensures it is running without blocking this script
            subprocess.run(
                ["docker-compose", "-f", self.compose_path, "up", "-d"],
                env=self.env, check=True
            )
            print("Docker container 'basekit_dev' is ready.")
        except subprocess.CalledProcessError as e:
            print(f"Error starting Docker: {e}")
            sys.exit(1)

    def run_in_container(self, command):
        """Helper to execute commands inside the running basekit container."""
        docker_cmd = [
            "docker", "exec", "-it", "basekit_dev", 
            "/bin/bash", "-c", f"source /opt/ros/humble/setup.bash && {command}"
        ]
        return subprocess.run(docker_cmd, env=self.env)

    def build_workspace(self):
        """Build the ROS 2 workspace inside Docker."""
        print("\n--- 3. Building ROS 2 Workspace ---")
        # We use symlink-install to avoid constant rebuilding of launch files
        build_cmd = "colcon build --symlink-install"
        result = self.run_in_container(build_cmd)
        
        if result.returncode != 0:
            print("Build failed! Check the errors above.")
            sys.exit(1)
        print("Build successful.")

    def launch_robot(self):
        """Launch the robot stack."""
        print("\n--- 4. Launching Open Agbot Stack ---")
        # Source the local workspace before launching
        launch_cmd = "source install/setup.bash && ros2 launch basekit_launch basekit_launch.py"
        self.run_in_container(launch_cmd)

if __name__ == "__main__":
    agbot = OpenAgBotDev()
    
    # 1. Setup local environment
    agbot.prepare_env()
    
    # 2. Start/Check Docker
    agbot.manage_docker()
    
    # 3. Build (Optional: you could add a --no-build flag)
    agbot.build_workspace()
    
    # 4. Launch
    try:
        agbot.launch_robot()
    except KeyboardInterrupt:
        print("\nShutdown signal received. Robot stopping...")
