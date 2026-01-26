#!/bin/bash
# VERSION: 6.3.0
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${YELLOW}=== OPEN AGBOT WORKSPACE ULTIMATE AUDIT V6.3 ===${NC}"

# ... [Keep Sections 1-4 from V6.2] ...

# 5. GPS PROTOCOL & MESSAGE DEEP DIVE
echo -e "\n5. GPS MESSAGE CONFIGURATION CHECK"
echo "------------------------------------------------"
UBLOX_YAML="src/ublox/ublox_gps/config/zed_f9p.yaml"
if [ -f "$UBLOX_YAML" ]; then
    echo -e "${BLUE}Inspecting $UBLOX_YAML for NACK-triggering keys:${NC}"
    # Check for high-frequency conflicts
    grep -E "meas_rate|nav_rate|ubx|inf|publish" "$UBLOX_YAML" | sed 's/^/  /'
    
    echo -e "\n${BLUE}Active Message Classes:${NC}"
    # If raw data is enabled at high rates, F9P will NACK
    grep -E "raw|sfrb|rxm|nav:" -A 5 "$UBLOX_YAML" | grep "all:" | sed 's/^/  /'
fi

# 6. LIVE DATA STREAM ANALYSIS
echo -e "\n6. LIVE TOPIC DATA CHECK"
echo "------------------------------------------------"
CONTAINER_NAME=$(docker ps --format '{{.Names}}' | grep -E "open_ag_runtime|open_agbot")
if [ ! -z "$CONTAINER_NAME" ]; then
    echo "Checking if GPS Fix is actually being published despite NACKs..."
    docker exec $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && timeout 3s ros2 topic echo /gps/fix --once" || echo -e "[${RED}✘${NC}] No data on /gps/fix yet."
else
    echo "Container not running. Start it to see live data."
fi

# 7. NUCLEAR RECOVERY (Updated to handle NACK loops)
echo -e "\n7. RECOVERY ACTIONS"
echo "------------------------------------------------"
read -p "Reset GPS Node & Apply Settings? [y/N]: " confirm
if [[ "$confirm" == [yY] ]]; then
    sudo systemctl stop gpsd 2>/dev/null
    sudo fuser -k /dev/ttyACM1 /dev/ttyACM2 2>/dev/null
    echo "Settings cleared. Restarting container..."
    docker restart $CONTAINER_NAME
fi

echo -e "\n${YELLOW}=== AUDIT COMPLETE ===${NC}"
