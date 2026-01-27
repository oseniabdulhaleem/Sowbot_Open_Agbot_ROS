# ==========================================
# STAGE 1: The Builder
# ==========================================
FROM ros:humble-ros-base AS builder

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV WORKSPACE=/workspace
WORKDIR $WORKSPACE

SHELL ["/bin/bash", "-c"]

# 1. Setup Environment & Reliability Layer
RUN sed -i 's|http://archive.ubuntu.com/ubuntu/|http://mirror.umd.edu/ubuntu/|g' /etc/apt/sources.list && \
    for i in {1..5}; do \
    apt-get update -y && \
    apt-get install -y --no-install-recommends \
    ca-certificates curl wget gnupg2 unzip git python3-pip python3-setuptools && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    break || (echo "Retry $i/5..." && sleep 5); \
    done

# 2. Add MongoDB & Install Heavy ROS Dependencies
RUN for i in {1..5}; do \
    curl -fsSL https://www.mongodb.org/static/pgp/server-6.0.asc | \
    gpg --dearmor --yes -o /usr/share/keyrings/mongodb-server-6.0.gpg && \
    echo "deb [ arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server-6.0.gpg ] https://repo.mongodb.org/apt/ubuntu jammy/mongodb-org/6.0 multiverse" | \
    tee /etc/apt/sources.list.d/mongodb-org-6.0.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends --fix-missing \
    mongodb-org-server mongodb-org-shell python3-pymongo \
    python3-serial udev libasio-dev python3-colcon-common-extensions \
    ros-humble-navigation2 ros-humble-nav2-msgs ros-humble-gps-msgs \
    ros-humble-interactive-markers ros-humble-tf-transformations \
    ros-humble-twist-mux ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher ros-humble-xacro \
    ros-humble-ublox-gps ros-humble-ublox-msgs \
    ros-humble-nmea-msgs ros-humble-rtcm-msgs \
    ros-humble-diagnostic-updater ros-humble-rmw-cyclonedds-cpp && \
    break || (echo "Retry $i/5 failing, waiting..." && sleep 10); \
    done

# 3. Python Logic & Firmware
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir nicegui lizard transforms3d

RUN mkdir -p /root/.lizard && cd /root && \
    LATEST_TAG=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget -q -O lizard.zip "https://github.com/zauberzeug/lizard/releases/download/${LATEST_TAG}/lizard_firmware_and_devtools_${LATEST_TAG}_esp32.zip" && \
    unzip -qo lizard.zip -d /root/.lizard && rm lizard.zip

# 4. Build ROS Workspace
COPY ./src ./src
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --executor sequential --parallel-workers 1

# ==========================================
# STAGE 2: The Runtime (Small Production Image)
# ==========================================
FROM ros:humble-ros-base
WORKDIR /workspace

# Copy build artifacts and firmware
COPY --from=builder /workspace/install /workspace/install
COPY --from=builder /root/.lizard /root/.lizard

# Configuration
ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV PYTHONPATH="${PYTHONPATH}:/workspace/install/basekit_ui/lib/python3.10/site-packages:/workspace/install/basekit_driver/lib/python3.10/site-packages"

# RE-ADD REPOS: Stage 2 needs the Mongo keys to install the shell!
RUN sed -i 's|http://archive.ubuntu.com/ubuntu/|http://mirror.umd.edu/ubuntu/|g' /etc/apt/sources.list && \
    apt-get update && apt-get install -y --no-install-recommends curl gnupg2 && \
    curl -fsSL https://www.mongodb.org/static/pgp/server-6.0.asc | \
    gpg --dearmor --yes -o /usr/share/keyrings/mongodb-server-6.0.gpg && \
    echo "deb [ arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server-6.0.gpg ] https://repo.mongodb.org/apt/ubuntu jammy/mongodb-org/6.0 multiverse" | \
    tee /etc/apt/sources.list.d/mongodb-org-6.0.list

# Install Runtime Libs
RUN apt-get update && apt-get install -y --no-install-recommends --fix-missing \
    python3-pip python3-serial mongodb-org-shell udev \
    ros-humble-ublox-gps ros-humble-nmea-msgs ros-humble-rtcm-msgs \
    ros-humble-tf-transformations ros-humble-rmw-cyclonedds-cpp && \
    python3 -m pip install --no-cache-dir nicegui lizard transforms3d && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Final Setup
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc && \
    usermod -a -G dialout root

CMD ["/bin/bash"]
