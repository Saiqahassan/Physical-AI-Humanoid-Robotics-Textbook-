# Quickstart: Digital Twin with Gazebo and Unity

**Date**: 2025-12-17

This document provides a quickstart guide for setting up and running the digital twin simulation.

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble/Iron
- Gazebo Fortress
- Unity LTS (with Linux build support)
- Docker

## Setup

1.  **Clone the repository**:
    ```bash
    git clone <repository-url>
    cd <repository-name>
    ```

2.  **Build the Docker image**:
    ```bash
    docker build -t digital-twin-env .
    ```

3.  **Run the Docker container**:
    ```bash
    docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY digital-twin-env
    ```

4.  **Build the ROS 2 workspace (inside the Docker container)**:
    ```bash
    cd /ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    ```

## Running the Simulation

1.  **Launch the Gazebo simulation (inside the Docker container)**:
    ```bash
    source install/setup.bash
    ros2 launch digital_twin spawn_humanoid.launch.py
    ```

2.  **Run the Unity visualization**:
    - Open the Unity project (`digital_twin_unity`) in the Unity Hub.
    - Open the main scene.
    - Refer to `digital_twin_unity/README.md` for detailed Unity setup and message generation instructions.
    - Press the "Play" button in the Unity editor to start the visualization.