# Dockerfile for Digital Twin Environment

# Use Ubuntu 22.04 as the base image
FROM ubuntu:22.04

# Set non-interactive frontend for package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential

# Install ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get install -y ros-humble-desktop

# Install Gazebo Fortress
RUN curl -sSL http://get.gazebosim.org | sh

# Install colcon and other ROS 2 tools
RUN apt-get update && apt-get install -y python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
RUN rosdep init && rosdep update

# --- Unity Installation ---
# Unity installation requires a graphical environment and a user account,
# so it's not straightforward to do in a Dockerfile.
# The recommended approach is to install Unity Hub on the host machine
# and use it to install the Unity Editor.
#
# Placeholder for Unity installation steps:
# 1. Download Unity Hub for Linux from the official Unity website.
# 2. Install Unity Hub on the host machine.
# 3. Use Unity Hub to install a compatible version of the Unity Editor (LTS).
# 4. Ensure the Linux build support module is installed.

# Create a ROS 2 workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Source ROS 2 setup file
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set the default command to bash
CMD ["/bin/bash"]
