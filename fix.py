import os
import shutil

def repair_setup_py():
    workspace = os.path.expanduser("~/open_agbot_ws")
    setup_py = os.path.join(workspace, "src/basekit_launch/setup.py")
    
    if not os.path.exists(setup_py):
        print(f"Error: Could not find {setup_py}")
        return

    print("--- Starting path correction for basekit_launch ---")

    with open(setup_py, 'r') as f:
        lines = f.readlines()

    updated_lines = []
    changed = False

    for line in lines:
        # Check for the nested path entry
        # Looks for (os.path.join('share', package_name, 'launch')
        if "os.path.join('share', package_name, 'launch')" in line:
            # Replace it with the flat path entry
            new_line = line.replace("'share', package_name, 'launch'", "'share', package_name")
            updated_lines.append(new_line)
            changed = True
            print("Action: Fixed nested launch directory mapping.")
        else:
            updated_lines.append(line)

    if changed:
        with open(setup_py, 'w') as f:
            f.writelines(updated_lines)
    else:
        print("Notice: setup.py path already appears correct or entry not found.")

    # Clean build folders to force the new configuration to take effect
    print("Action: Clearing build, install, and log folders...")
    for folder in ['build', 'install', 'log']:
        path = os.path.join(workspace, folder)
        if os.path.exists(path):
            shutil.rmtree(path)

    print("--- Repair complete. Run: python3 openagbot-dev.py ---")

if __name__ == "__main__":
    repair_setup_py()
