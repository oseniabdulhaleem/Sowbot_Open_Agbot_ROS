import os
import subprocess
import sys

class AgBotDevKit:
    def __init__(self):
        self.workspace_root = os.path.dirname(os.path.abspath(__file__))
        self.compose_path = os.path.join(self.workspace_root, "docker/docker-compose.yml")
        self.uid = os.getuid()
        self.gid = os.getgid()

    def run_step(self, label, command, shell=False):
        """Executes a command and handles potential errors."""
        print(f"STEP: {label}")
        try:
            env = os.environ.copy()
            env["UID"] = str(self.uid)
            env["GID"] = str(self.gid)
            
            subprocess.run(command, shell=shell, check=True, env=env, cwd=self.workspace_root)
        except subprocess.CalledProcessError as e:
            print(f"FAILURE: Error during {label}")
            print(f"Details: {e}")
            sys.exit(1)

    def setup_workspace(self):
        print("--- Open AgBot DevKit Initialization ---")

        # Unlock serial communication for sensors
        self.run_step("Unlocking serial ports", "sudo chmod 666 /dev/ttyUSB* /dev/ttyACM* || true", shell=True)

        # Clear and recreate build directories to prevent root ownership
        print("STEP: Resetting build, install, and log directories")
        for folder in ['build', 'install', 'log']:
            path = os.path.join(self.workspace_root, folder)
            if os.path.exists(path):
                subprocess.run(["sudo", "rm", "-rf", path], check=True)
            # Creating as the script user, not sudo
            os.makedirs(path, exist_ok=True)
        
        # Ensure all workspace files belong to the host user
        self.run_step("Reclaiming file ownership", 
                      ["sudo", "chown", "-R", f"{self.uid}:{self.gid}", self.workspace_root])

    def build_and_launch(self):
        # Align image name with existing local images
        self.run_step("Syncing Docker image tags", 
                      "docker tag open-agbot-devkit:latest openagbot-basekit:latest || true", shell=True)

        # Compile workspace using colcon inside the container
        build_cmd = [
            "docker-compose", "-f", self.compose_path, "run", "--rm", "basekit",
            "/bin/bash", "-c", "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
        ]
        self.run_step("Building ROS 2 packages", build_cmd)

        # Start the robot services
        launch_cmd = ["docker-compose", "-f", self.compose_path, "up"]
        self.run_step("Launching robot services", launch_cmd)

if __name__ == "__main__":
    devkit = AgBotDevKit()
    devkit.setup_workspace()
    devkit.build_and_launch()
