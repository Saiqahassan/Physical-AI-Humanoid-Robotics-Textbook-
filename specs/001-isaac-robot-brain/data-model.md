# Data Model: Module 3 — The AI-Robot Brain (NVIDIA Isaac™)

## Overview

This document outlines the key entities and data structures involved in "Module 3 — The AI-Robot Brain (NVIDIA Isaac™)", focusing on their attributes and relationships within the context of simulation, perception, and navigation for humanoid robots.

## Entities

### 1. Humanoid Robot Model

*   **Description**: Represents the bipedal robot within the NVIDIA Isaac Sim environment. This is a virtual model that interacts with the simulated physics and environment.
*   **Key Attributes**:
    *   `joints`: Array of joint states (position, velocity, effort).
    *   `pose`: 3D position and orientation (e.g., `x, y, z, roll, pitch, yaw` or quaternion).
    *   `sensors`: Collection of attached sensors, each with its type and data stream.
        *   `camera`: RGB, Depth, Segmentation data, intrinsics/extrinsics.
        *   `imu`: Linear acceleration, angular velocity, orientation.
        *   `lidar`: Point cloud data.
    *   `actuators`: Interfaces for controlling robot movement (e.g., joint commands).
*   **Relationships**: Interacts with `Simulated Environment`, generates data for `Synthetic Dataset` and `VSLAM Pipeline`. Controlled by `Navigation Stack (Nav2)`.

### 2. Simulated Environment

*   **Description**: The virtual world constructed within NVIDIA Isaac Sim where the `Humanoid Robot Model` operates.
*   **Key Attributes**:
    *   `scene_geometry`: 3D models of static objects (walls, furniture) and terrain.
    *   `dynamic_objects`: 3D models of moving objects (e.g., other robots, obstacles).
    *   `lighting_conditions`: Ambient light, directional lights, point lights.
    *   `physics_parameters`: Gravity, friction coefficients, contact properties.
*   **Relationships**: Hosts `Humanoid Robot Model`, provides context for `Synthetic Dataset` generation.

### 3. Synthetic Dataset

*   **Description**: A collection of high-fidelity sensor data and corresponding ground truth generated programmatically from NVIDIA Isaac Sim using tools like Isaac Sim Replicator.
*   **Key Attributes**:
    *   `rgb_images`: Photorealistic color images.
    *   `depth_images`: Per-pixel depth information.
    *   `segmentation_masks`: Pixel-wise labels for objects.
    *   `bounding_boxes`: 2D/3D coordinates for object detection.
    *   `object_poses`: Ground truth 3D positions and orientations of all objects in the scene.
    *   `camera_intrinsics/extrinsics`: Camera calibration parameters.
    *   `robot_ground_truth_pose`: The actual pose of the robot in the simulation.
*   **Relationships**: Generated from `Simulated Environment` and `Humanoid Robot Model`. Used as input for training AI models (not directly part of this module's processing pipeline, but foundational).

### 4. VSLAM Pipeline

*   **Description**: A software module, primarily leveraging Isaac ROS packages, that processes visual and inertial data streams from the `Humanoid Robot Model` to simultaneously estimate the robot's `pose` and build a `map` of the `Simulated Environment`.
*   **Key Attributes**:
    *   `input_streams`: Camera images (RGB/Depth), IMU data.
    *   `estimated_robot_pose`: Real-time 3D position and orientation output.
    *   `map_data`: Sparse feature map or dense occupancy grid/3D reconstruction.
    *   `confidence_metrics`: Estimation uncertainty or quality indicators.
*   **Relationships**: Consumes data from `Humanoid Robot Model` sensors. Produces `estimated_robot_pose` and `map_data` used by `Navigation Stack (Nav2)`.

### 5. Navigation Stack (Nav2)

*   **Description**: A collection of ROS 2 packages that enable autonomous mobile robot navigation, specifically adapted for the `Humanoid Robot Model`.
*   **Key Attributes**:
    *   `input_pose`: Current robot pose, typically from `VSLAM Pipeline`.
    *   `input_map`: Environmental map, typically from `VSLAM Pipeline` or pre-built.
    *   `goal_pose`: Desired target 3D position and orientation for the robot.
    *   `global_plan`: A high-level path from `input_pose` to `goal_pose`.
    *   `local_plan`: A short-term, dynamically generated path to avoid immediate obstacles.
    *   `velocity_commands`: Actuator commands (e.g., linear/angular velocities) sent to `Humanoid Robot Model`.
    *   `costmaps`: 2D/3D grid representations of the environment, indicating traversability and obstacles.
*   **Relationships**: Consumes `estimated_robot_pose` and `map_data` from `VSLAM Pipeline`. Sends `velocity_commands` to `Humanoid Robot Model`.