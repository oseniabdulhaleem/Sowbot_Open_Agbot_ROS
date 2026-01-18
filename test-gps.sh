#!/bin/bash

# Configuration
GPS_DEVICE="/dev/ttyACM0"
BAUD_RATE="115200"

echo "--- 🚜 AgBot ThinkPad GPS Setup & Test ---"

# 1. Clean up broken ROS repos (Plucky Fix)
echo "[1/4] Checking Repositories..."
if [ -f /etc/apt/sources.list.d/ros2.list ]; then
    sudo mv /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2.list.bak
    echo "   - Temporarily disabled broken ROS repo."
fi

# 2. Install/Update gpsd without ROS interference
sudo apt-get update -qq
sudo apt-get install -y gpsd gpsd-clients fuser

# 3. Kill all blockers (The "Nuclear" Clean)
echo "[2/4] Killing system locks..."
sudo systemctl stop gpsd.socket 2>/dev/null
sudo systemctl disable gpsd.socket 2>/dev/null
sudo systemctl stop gpsd 2>/dev/null
sudo fuser -k 2947/tcp 2>/dev/null
sudo fuser -k $GPS_DEVICE 2>/dev/null

# 4. Hardware Prep
echo "[3/4] Prepping Hardware..."
sudo chmod 666 $GPS_DEVICE
sudo stty -F $GPS_DEVICE $BAUD_RATE raw -echo

# 5. Launch and Test
echo "[4/4] Starting GPSD & Monitoring..."
echo "------------------------------------------------"
echo "TIP: Keep this window open. Open a NEW tab and run 'cgps -s'"
echo "------------------------------------------------"

# Launch gpsd in foreground with listen-on-all-interfaces (-G)
# This allows Docker to talk to it.
sudo gpsd -N -D 3 -G $GPS_DEVICE
