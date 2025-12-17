# Unity Project Setup for Digital Twin

This document provides instructions for setting up the Unity project to visualize the digital twin.

## 1. Create the Unity Project

If you haven't already, create a new 3D Unity project named `digital_twin_unity` in this directory using Unity Hub.

## 2. Set up `ROS2 For Unity`

1.  **Download `ROS2 For Unity`**:
    *   Go to the `ROS2 For Unity` GitHub repository: `https://github.com/RobotecAI/ros2_for_unity`
    *   Download the latest `.unitypackage` from the releases page.

2.  **Import into Unity Project**:
    *   Open your `digital_twin_unity` project in the Unity Editor.
    *   In Unity, go to `Assets > Import Package > Custom Package...`.
    *   Select the downloaded `.unitypackage` file and import all assets.

3.  **Configure `ROS2 For Unity`**:
    *   Follow the official `ROS2 For Unity` documentation for initial setup and configuration steps within Unity. This usually involves:
        *   Setting up the `ROS2Manager` in your scene.
        *   Ensuring the correct ROS 2 distribution is sourced on your system before running Unity (if using overlay mode).

## 3. Generate ROS 2 Messages in Unity

1.  **Copy ROS 2 Message Definitions**:
    *   You will need to copy the relevant ROS 2 message definitions (`.msg` files for `sensor_msgs`, `geometry_msgs`, etc.) from your ROS 2 installation to a specific folder within your Unity project (e.g., `Assets/ROS2Messages`).
    *   For `sensor_msgs/JointState`, `tf2_msgs/TFMessage`, `sensor_msgs/LaserScan`, `sensor_msgs/Image`, `sensor_msgs/Imu`, `geometry_msgs/Twist`.

2.  **Generate C# Message Files**:
    *   `ROS2 For Unity` provides tools to generate C# message classes from `.msg` files. Refer to its documentation for the exact process, which usually involves a Unity Editor menu option.

## 4. Next Steps

After setting up `ROS2 For Unity` and generating messages, you can proceed with implementing the C# scripts to subscribe to ROS 2 topics and update the robot's visualization.
