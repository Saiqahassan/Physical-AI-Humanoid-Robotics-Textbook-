# ROS 2 Topics: Digital Twin with Gazebo and Unity

**Date**: 2025-12-17

This document describes the ROS 2 topics used for communication between the Gazebo simulation, the Unity visualization, and the robot control nodes.

## Published Topics

- **`/joint_states`**
  - **Message Type**: `sensor_msgs/JointState`
  - **Description**: Publishes the current state of the robot's joints (position, velocity, and effort).

- **`/tf`**
  - **Message Type**: `tf2_msgs/TFMessage`
  - **Description**: Publishes the transformations between different coordinate frames in the simulation.

- **`/lidar`**
  - **Message Type**: `sensor_msgs/LaserScan`
  - **Description**: Publishes the 2D or 3D laser scan data from the simulated LiDAR sensor.

- **`/depth_camera`**
  - **Message Type**: `sensor_msgs/Image`
  - **Description**: Publishes the depth image from the simulated depth camera.

- **`/imu`**
  - **Message Type**: `sensor_msgs/Imu`
  - **Description**: Publishes the orientation, angular velocity, and linear acceleration data from the simulated IMU.

## Subscribed Topics

- **`/cmd_vel`**
  - **Message Type**: `geometry_msgs/Twist`
  - **Description**: Subscribes to velocity commands to control the base of the robot.
