# Data Model: Digital Twin with Gazebo and Unity

**Date**: 2025-12-17

This document describes the key data entities for the "Digital Twin with Gazebo and Unity" feature.

## 1. Humanoid Robot Model

**Description**: Represents the physical structure of the humanoid robot. This model is defined using the Unified Robot Description Format (URDF) or the Simulation Description Format (SDF).

**Attributes**:
- **Links**: The rigid bodies of the robot.
- **Joints**: The connections between links, which allow for relative motion.
- **Sensors**: The sensors attached to the robot, such as LiDAR, cameras, and IMUs.
- **Collision Meshes**: The simplified geometries used for collision detection.
- **Inertial Properties**: The mass and inertia of each link.

## 2. Gazebo World

**Description**: The simulation environment in Gazebo. This is defined in a `.world` file.

**Attributes**:
- **Physics Properties**: The physics engine settings, such as gravity, friction, and constraints.
- **Objects**: The static and dynamic objects in the environment.
- **Lighting**: The lighting sources in the environment.

## 3. Unity Scene

**Description**: The high-fidelity visualization environment in Unity.

**Attributes**:
- **3D Models**: The detailed 3D models of the robot and the environment.
- **Lighting and Effects**: The lighting, shadows, and other visual effects that create a realistic scene.
- **Scripts**: The C# scripts that control the behavior of the scene, including the synchronization with Gazebo.

## 4. ROS 2 Topics

**Description**: The communication channels used for controlling the robot and receiving sensor data. These are defined by the ROS 2 message types.

**Key Topics**:
- `/joint_states` (`sensor_msgs/JointState`)
- `/tf` (`tf2_msgs/TFMessage`)
- `/lidar` (`sensor_msgs/LaserScan`)
- `/depth_camera` (`sensor_msgs/Image`)
- `/imu` (`sensor_msgs/Imu`)
- `/cmd_vel` (`geometry_msgs/Twist`)
