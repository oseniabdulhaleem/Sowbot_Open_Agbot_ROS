import os
import subprocess
import sys

def run_cmd(cmd, description, can_fail=False):
    print(f"\n🚀 {description}...")
    try:
        subprocess.run(cmd, shell=True, check=not can_fail)
    except subprocess.CalledProcessError as e:
        if not can_fail:
            print(f"❌ Error during: {description}")
            print(f"Details: {e}")
            sys.exit(1)
        else:
            print(f"⚠️  Note: {description} skipped or failed (non-critical).")

def main():
    print("--- 🤖 Open AgBot DevKit Proper Setup ---")

    # 1. Unlock Hardware Ports (Marked as can_fail if hardware is unplugged)
    run_cmd("sudo chmod 666 /dev/ttyACM* || true", "Unlocking serial ports", can_fail=True)

    # 2. Fix setup.py naming mismatch
    setup_py_path = "src/basekit_ui/setup.py"
    if os.path.exists(setup_py_path):
        cmd = "sed -i \"s/['\\\"]ui_node.*=['\\\"]/ 'basekit_ui_node =/g\" " + setup_py_path
        run_cmd(cmd, "Correcting entry_points in setup.py")

    # 3. Fix Shebang & Line Endings
    ui_node_path = "src/basekit_ui/basekit_ui/ui_node.py"
    if os.path.exists(ui_node_path):
        run_cmd(f"sed -i '1s|^.*$|#!/usr/bin/env python3|' {ui_node_path}", "Injecting Python Shebang")
        run_cmd(f"chmod +x {ui_node_path}", "Making UI node executable")
        run_cmd(f"sed -i 's/\\r$//' {ui_node_path}", "Removing Windows line endings")

    # 4. Wipe old build artifacts (USING SUDO to fix your permission error)
    run_cmd("sudo rm -rf build/ install/ log/", "Wiping old build/install folders with sudo")

    # 5. Build the Docker Image
    run_cmd("docker-compose -f docker/docker-compose.yml build", "Building Docker image")

    # 6. INTERNAL BUILD
    build_cmd = (
        "docker-compose -f docker/docker-compose.yml run --rm basekit /bin/bash -c "
        "'source /opt/ros/humble/setup.bash && colcon build --symlink-install'"
    )
    run_cmd(build_cmd, "Compiling ROS 2 workspace inside container")

    # 7. Start the Robot
    run_cmd("docker-compose -f docker/docker-compose.yml up -d", "Launching AgBot Background Services")

    print("\n" + "="*40)
    print("✅ SUCCESS: Robot software is ready!")
    print("🌎 ACCESS WEB UI: http://localhost:8080")
    print("="*40)

if __name__ == "__main__":
    main()
