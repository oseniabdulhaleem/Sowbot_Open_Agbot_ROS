FROM ros:humble-ros-base
ENV DEBIAN_FRONTEND=noninteractive
ENV WORKSPACE=/workspace
WORKDIR $WORKSPACE

# Install everything via APT that we can
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg2 build-essential cmake python3-pip \
    libasio-dev libgeographic-dev python3-colcon-common-extensions \
    ros-humble-rmw-cyclonedds-cpp python3-serial udev \
    ros-humble-diagnostic-updater \
    ros-humble-ublox-msgs \
    ros-humble-xacro \
    ros-humble-tf-transformations \
    python3-transforms3d

RUN curl -fsSL https://www.mongodb.org/static/pgp/server-6.0.asc | \
    gpg --dearmor --yes -o /usr/share/keyrings/mongodb-server-6.0.gpg && \
    echo "deb [ arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server-6.0.gpg ] https://repo.mongodb.org/apt/ubuntu jammy/mongodb-org/6.0 multiverse" | \
    tee /etc/apt/sources.list.d/mongodb-org-6.0.list

RUN apt-get update && apt-get install -y mongodb-org-shell python3-pymongo

# Only install UI/Logic libs via PIP
RUN python3 -m pip install --no-cache-dir nicegui lizard

# This ensures Python can find the ROS-installed transformations package
ENV PYTHONPATH="${PYTHONPATH}:/opt/ros/humble/lib/python3.10/site-packages"
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
