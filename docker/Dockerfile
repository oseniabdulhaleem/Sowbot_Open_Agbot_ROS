# Use official ROS 2 Humble base
FROM ros:humble-ros-base

# 1. Setup Environment & Shell
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Ensure the Lizard tools and the built UI packages are in the Python path
ENV PYTHONPATH="/root/.lizard:/workspace/install/basekit_ui/lib/python3.10/site-packages"
SHELL ["/bin/bash", "-c"]

# 2. Hardening Apt & Base Tools
# We switch to Azure mirror and update early to stabilize the connection
RUN sed -i 's/archive.ubuntu.com/azure.archive.ubuntu.com/g' /etc/apt/sources.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    wget curl ca-certificates unzip git nano python3-pip software-properties-common && \
    add-apt-repository universe && \
    apt-get update

# 3. Robust Dependency Management (The Timeout Fix)
# We wrap the entire install in a retry loop to survive 408 Time-outs on OpenCV
RUN for i in {1..5}; do \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-serial udev libasio-dev python3-colcon-common-extensions \
    python3-setuptools python3-wheel \
    ros-humble-rtcm-msgs ros-humble-usb-cam ros-humble-xacro \
    ros-humble-nmea-msgs ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher ros-humble-controller-manager \
    ros-humble-diff-drive-controller ros-humble-twist-mux \
    ros-humble-rmw-cyclonedds-cpp ros-humble-iceoryx-binding-c && \
    break || (echo "Retry $i/5 failing, waiting..." && sleep 10); \
    done && rm -rf /var/lib/apt/lists/*

# 4. Core Python Dependencies (NiceGUI / AgBot Stack)
# Installing these before copying the source code speeds up future builds
RUN pip3 install nicegui lizard

# 5. Lizard Espresso Firmware & Devtools
RUN mkdir -p /root/.lizard && cd /root && \
    LATEST_TAG=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget -O lizard.zip "https://github.com/zauberzeug/lizard/releases/download/${LATEST_TAG}/lizard_firmware_and_devtools_${LATEST_TAG}_esp32.zip" && \
    unzip -o lizard.zip -d /root/.lizard && rm lizard.zip && \
    chmod +x /root/.lizard/espresso.py

# 6. Workspace Setup
WORKDIR /workspace
# Copying the source code from the local context
COPY ./src ./src

# 7. Staged Build (Proper ROS 2 Dependency Order)
# Building sequentially prevents ublox_gps from failing if msgs aren't ready
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select ublox_serialization && \
    . install/setup.sh && \
    colcon build --symlink-install --packages-select ublox_msgs && \
    . install/setup.sh && \
    colcon build --symlink-install --parallel-workers 1

# 8. Environment Sourcing
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

# 9. Clean Entrypoint (Bypass loop-crashes)
# Launch is now handled by docker-compose for better stability
CMD ["/bin/bash"]
