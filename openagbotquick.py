import os
import subprocess
import sys

def run_cmd(cmd, description, can_fail=False):
    print(f"\n🚀 {description}...")
    try:
        # We use shell=True to handle wildcards and piping
        subprocess.run(cmd, shell=True, check=not can_fail)
    except subprocess.CalledProcessError as e:
        if not can_fail:
            print(f"❌ ERROR during: {description}")
            print(f"Details: {e}")
            sys.exit(1)
        else:
            print(f"⚠️  WARNING: {description} failed (non-critical).")

def main():
    print("--- 🤖 Open AgBot DevKit Proper Setup ---")

    # 1. Unlock Hardware Ports
    # can_fail=True because devkits might not be plugged in during software-only testing
    run_cmd("sudo chmod 666 /dev/ttyACM* || true", "Unlocking serial ports", can_fail=True)

    # 2. Fix setup.py naming mismatch
    # Ensures ROS finds 'basekit_ui_node' even if the file is named 'ui_node.py'
    setup_py_path = "src/basekit_ui/setup.py"
    if os.path.exists(setup_py_path):
        cmd = "sed -i \"s/['\\\"]ui_node.*=['\\\"]/ 'basekit_ui_node =/g\" " + setup_py_path
        run_cmd(cmd, "Correcting entry_points in setup.py")

    # 3. Fix Shebang & Line Endings (The 'OSError 8' killer)
    ui_node_path = "src/basekit_ui/basekit_ui/ui_node.py"
    if os.path.exists(ui_node_path):
        run_cmd(f"sed -i '1s|^.*$|#!/usr/bin/env python3|' {ui_node_path}", "Injecting Python Shebang")
        run_cmd(f"chmod +x {ui_node_path}", "Making UI node executable")
        run_cmd(f"sed -i 's/\\r$//' {ui_node_path}", "Removing Windows line endings")

    # 4. Deep Clean
    # Uses sudo to wipe root-owned build files from previous Docker runs
    run_cmd("sudo rm -rf build/ install/ log/", "Wiping old build artifacts with sudo")

    # 5. Docker Image Build
    run_cmd("docker-compose -f docker/docker-compose.yml build", "Building Docker image")

    # 6. INTERNAL WORKSPACE BUILD (The Mission Critical Step)
    # We use --entrypoint '' to bypass the container's internal setup.bash check.
    # This allows us to create the install folder that the container usually expects.
    build_cmd = (
        "docker-compose -f docker/docker-compose.yml run --rm "
        "--entrypoint '' "
        "basekit /bin/bash -c "
        "\"source /opt/ros/humble/setup.bash && colcon build --symlink-install\""
    )
    run_cmd(build_cmd, "Compiling ROS 2 workspace inside container")

    # 7. Start the Background Services
    run_cmd("docker-compose -f docker/docker-compose.yml up -d", "Launching AgBot Background Services")

    print("\n" + "="*50)
    print("✅ SUCCESS: Robot software is running!")
    print("🌎 ACCESS WEB UI: http://localhost:8080")
    print("📋 VIEW LOGS:     docker logs -f open_agbot_basekit")
    print("="*50)

if __name__ == "__main__":
    main()
