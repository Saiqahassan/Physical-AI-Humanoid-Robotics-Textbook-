# Contracts: Module 3 — The AI-Robot Brain (NVIDIA Isaac™)

## Overview

This module focuses on an educational content delivery, demonstrating the integration and usage of NVIDIA Isaac Sim, Isaac ROS, and Nav2 for humanoid robotics perception, simulation, and navigation. As such, it does not expose traditional RESTful or GraphQL APIs for external consumption.

## "Contracts" in this context

The "contracts" within this module are defined by:

1.  **ROS 2 Interfaces**: The standard ROS 2 message types, services, and actions used for inter-node communication (e.g., sensor_msgs/Image, geometry_msgs/PoseStamped, nav_msgs/Odometry, Nav2 action protocols). These govern the data exchange between Isaac Sim (via ROS 2 Bridge), Isaac ROS packages, and Nav2.
2.  **NVIDIA Isaac Sim Python APIs**: The Python APIs (e.g., Omniverse Kit, `omni.isaac.core`, `omni.isaac.synthetic_utils`) used to programmatically control the simulation, generate synthetic data, and interact with the virtual robot.
3.  **Isaac ROS & Nav2 Configuration Parameters**: The various YAML configuration files and launch arguments that define the behavior and tuning of the Isaac ROS perception pipelines and the Nav2 navigation stack.

These interfaces, APIs, and configurations serve as the implicit "contracts" for students developing and integrating components within the module. Formal OpenAPI/GraphQL schemas are not applicable for this educational and integration-focused module.