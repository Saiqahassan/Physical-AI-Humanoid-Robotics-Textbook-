# Quickstart: Module 3 — The AI-Robot Brain (NVIDIA Isaac™)

## Overview

This quickstart guide provides a rapid setup and execution path for getting started with the NVIDIA Isaac Sim, Isaac ROS, and Nav2 module for humanoid robotics. It covers the essential steps to prepare your environment and run the foundational examples.

## Prerequisites

Before proceeding, ensure you have:

*   **Operating System**: Ubuntu 22.04 LTS (recommended).
*   **ROS 2**: Humble or Iron installed (refer to official ROS 2 documentation for installation).
*   **NVIDIA Graphics Driver**: Latest proprietary NVIDIA driver.
*   **Docker & NVIDIA Container Toolkit**: Installed and configured for GPU acceleration.
*   **NVIDIA Isaac Sim 4.x**: Installed and accessible (refer to official NVIDIA documentation for installation). Ensure you can launch Isaac Sim and access its Python environment.

## 1. Environment Setup

It is highly recommended to use a dedicated development container for this module to ensure all dependencies are correctly isolated and configured.

1.  **Clone the Repository**:
    ```bash
    git clone [REPOSITORY_URL]
    cd [REPOSITORY_NAME]
    ```
    *(Replace `[REPOSITORY_URL]` and `[REPOSITORY_NAME]` with actual values once known)*

2.  **Launch Development Container (Example)**:
    *(This step assumes a Docker-based development environment setup, common in robotics)*
    ```bash
    # Example command, actual command may vary based on project setup
    docker compose up -d
    docker exec -it <container_name> bash
    ```

3.  **Install ROS 2 Dependencies**:
    Inside your ROS 2 workspace (likely within the container), ensure all necessary packages are installed.
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

## 2. Running Your First Isaac Sim Example (Synthetic Data Generation)

This example will demonstrate launching Isaac Sim and using its Python API to generate a simple synthetic dataset.

1.  **Launch Isaac Sim**: Start Isaac Sim from your system's launcher or by running `isaac-sim.sh` in the Isaac Sim installation directory.
    *(Alternatively, if running headless within a container, a specific launch command will be provided later)*

2.  **Open the Synthetic Data Generation Script**:
    Navigate to the `robotics-code/isaac-sims/synthetic_data_generation/` directory (example path) and open `generate_dataset.py`.

3.  **Execute the Script**:
    *(This typically involves running the script from within the Isaac Sim Script Editor or via its Python environment)*
    ```bash
    # From Isaac Sim's Script Editor: Run Script
    # Or from a terminal with Isaac Sim's Python env activated:
    python generate_dataset.py
    ```
    Observe the synthetic data being generated and saved to a specified output directory.

## 3. Running Your First Isaac ROS Example (VSLAM)

This example will demonstrate deploying a GPU-accelerated VSLAM pipeline using Isaac ROS.

1.  **Ensure Isaac Sim is Running**: Launch Isaac Sim with the appropriate humanoid robot scene and ROS 2 Bridge enabled.

2.  **Launch Isaac ROS VSLAM Node**:
    ```bash
    # Example launch command for an Isaac ROS VSLAM package
    ros2 launch isaac_ros_visual_slam visual_slam.launch.py # ... (add necessary args)
    ```
    Verify that the VSLAM node is publishing robot poses and map data.

## 4. Running Your First Nav2 Example (Humanoid Navigation)

This example will demonstrate basic autonomous navigation for the humanoid robot using Nav2.

1.  **Ensure Isaac Sim and VSLAM are Running**: Isaac Sim with robot and environment, and Isaac ROS VSLAM providing localization.

2.  **Launch Nav2 Stack**:
    ```bash
    # Example launch command for Nav2
    ros2 launch nav2_bringup bringup_launch.py # ... (add necessary args for humanoid)
    ```

3.  **Set a Navigation Goal**: Use RViz or a command-line tool to set a goal pose for the humanoid robot.
    Observe the robot planning and executing its path.

*(Detailed configuration files and specific example paths will be provided in the respective chapter sections.)*