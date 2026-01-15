FROM ros:humble-ros-base

# 1. Network & Env Setup
RUN echo 'Acquire::http::Pipeline-Depth "0"; Acquire::Retries "100";' > /etc/apt/apt.conf.d/99-resilience
ENV VIRTUAL_ENV=/opt/agbot_venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
ENV PYTHONPATH="/root/.lizard:$PYTHONPATH"
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 2. Resilient System Install
RUN apt-get update && \
    for i in {1..10}; do \
    apt-get install -y --no-install-recommends \
    python3-pip python3-venv libasio-dev wget unzip git nano \
    ros-humble-usb-cam ros-humble-xacro ros-humble-nmea-msgs \
    ros-humble-rtcm-msgs ros-humble-diagnostic-updater \
    ros-humble-twist-mux ros-humble-rmw-cyclonedds-cpp && \
    break || sleep 5; done && rm -rf /var/lib/apt/lists/*

# 3. Lizard Espresso & Venv
RUN mkdir -p /root/.lizard && cd /root && \
    LATEST=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget https://github.com/zauberzeug/lizard/releases/download/${LATEST}/lizard_firmware_and_devtools_${LATEST}_esp32.zip && \
    unzip *.zip -d /root/.lizard && rm *.zip

RUN python3 -m venv $VIRTUAL_ENV && \
    pip install --no-cache-dir nicegui pyserial lizard-lib

# 4. Workspace Build (The Sequential Logic)
WORKDIR /workspace
COPY ./src ./src
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select ublox_serialization && \
    . install/setup.sh && \
    colcon build --symlink-install --packages-select ublox_msgs && \
    . install/setup.sh && \
    colcon build --symlink-install --parallel-workers 1

# 5. Interactive Shell Setup
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source $VIRTUAL_ENV/bin/activate" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.sh && source $VIRTUAL_ENV/bin/activate && source install/setup.bash && ros2 launch basekit_launch robot.launch.py"]
