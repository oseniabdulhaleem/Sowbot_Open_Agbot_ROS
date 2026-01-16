import os
import subprocess

def get_output(cmd):
    try:
        return subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT).decode().strip()
    except subprocess.CalledProcessError as e:
        return f"Error: {e.output.decode()}"

print("--- 🕵️ ROS 2 Build Debug Collector ---")

# 1. Check Directory Structure
print("\n📂 [1] Checking Source Structure:")
print(get_output("find src/basekit_ui -maxdepth 2"))

# 2. Check setup.py Content
print("\nscript [2] setup.py Entry Points:")
print(get_output("grep -A 5 'entry_points' src/basekit_ui/setup.py"))

# 3. Check the Install Folder (Where the error is happening)
print("\n🚀 [3] Looking for the executable in 'install':")
install_path = "install/basekit_ui/lib/basekit_ui"
if os.path.exists(install_path):
    print(get_output(f"ls -l {install_path}"))
else:
    print(f"❌ Path {install_path} does not exist!")

# 4. Check for Hidden Windows Line Endings (CRLF)
print("\n📄 [4] Checking for hidden characters in ui_node.py:")
print(get_output("file src/basekit_ui/basekit_ui/ui_node.py"))

# 5. Check Docker internal environment
print("\n🐳 [5] Checking Docker internal PATH and Entrypoint:")
docker_cmd = (
    "docker-compose -f docker/docker-compose.yml run --rm --entrypoint '' "
    "basekit /bin/bash -c 'source /opt/ros/humble/setup.bash && env | grep ROS'"
)
print(get_output(docker_cmd))

# 6. Check for Shebang in the source
print("\n#! [6] Checking Shebang:")
print(get_output("head -n 1 src/basekit_ui/basekit_ui/ui_node.py"))

print("\n--- 🏁 Debug Info Gathering Complete ---")
