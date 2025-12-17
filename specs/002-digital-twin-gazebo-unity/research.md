# Research: Digital Twin with Gazebo and Unity

**Date**: 2025-12-17

This document summarizes the research conducted for the "Digital Twin with Gazebo and Unity" feature.

## 1. Unity Testing Framework for ROS 2 Integration

**Decision**: A hybrid approach will be used for testing, combining Unity's built-in Test Runner with standard ROS 2 testing frameworks.

**Rationale**: There is no single, dedicated framework for end-to-end testing of ROS 2 and Unity integration. The most robust approach involves testing each part of the system with its native tools.

- **Unity-side (C#):** Unity's Test Runner will be used to write unit and integration tests for all C# scripts that interact with ROS 2 messages or manage the simulation state.
- **ROS 2-side (Python/C++):** `pytest` and `gtest` will be used for unit testing of ROS 2 nodes. `launch_testing` will be used for integration testing of the ROS 2 nodes.
- **Integration:** The `ROS2 For Unity` package will be the primary choice for integration, as it provides high-performance, native ROS 2 nodes in Unity.

**Alternatives considered**: 
- Using only Unity's Test Runner and mocking ROS 2 messages. This was rejected because it would not provide adequate testing of the ROS 2 integration.
- Using only ROS 2 testing frameworks and treating the Unity simulation as a black box. This was rejected as it would not allow for testing of the Unity-specific logic.

## 2. Best Practices for Integrating ROS 2 with Unity

**Decision**: The integration will follow these best practices:
- Use a high-performance, native ROS 2 integration solution (**ROS2 For Unity**).
- Utilize ROS 2 features such as QoS settings and native time to ensure robust communication.
- Automate the generation of C# message scripts from ROS `.msg` files.
- Use Unity's **URDF Importer** to streamline the process of importing robot models.
- Use **Docker** to create a reproducible development and simulation environment.

**Rationale**: These practices will ensure a high-performance, maintainable, and reproducible integration between ROS 2 and Unity.

## 3. Best Practices for Creating Reproducible Simulations in Gazebo

**Decision**: The following practices will be implemented to improve the reproducibility of the Gazebo simulations:
- Use **lockstep mode** to synchronize the simulation and control loops.
- Set a **random seed** at the start of each simulation to control any random processes.
- Deactivate or seed sensor noise models during testing to ensure deterministic sensor data.
- Use the **DART** physics engine, which is the default in newer Gazebo versions and may offer better determinism than ODE.
- Simplify robot models and simulation environments where possible to maintain consistent performance.

**Rationale**: While perfect determinism is difficult to achieve in Gazebo, these practices will significantly improve the reproducibility of the simulations, which is a key requirement for this project.
