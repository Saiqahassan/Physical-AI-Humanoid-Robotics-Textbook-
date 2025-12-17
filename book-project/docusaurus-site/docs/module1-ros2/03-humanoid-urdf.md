# Building a Basic Humanoid URDF

The Unified Robot Description Format (URDF) is an XML format used in ROS to describe all aspects of a robot. This includes its kinematic and dynamic properties, visual appearance, and collision behavior. In this chapter, you will learn the fundamentals of URDF by building a simple humanoid robot model.

## URDF Fundamentals

*   **Links**: Represent the rigid bodies of the robot (e.g., torso, head, limbs).
*   **Joints**: Connect links and define their relative motion (e.g., revolute, prismatic).
*   **Visuals**: Describe the geometric and material properties for rendering.
*   **Collisions**: Define the geometric properties for collision detection.

## Creating a Simple Humanoid URDF

We will construct a basic humanoid robot model consisting of a torso, head, and two arms. This will introduce you to defining links, joints, and their properties. The full URDF model can be found at `book-project/robotics-code/urdf-models/simple_humanoid.urdf`.

## Visualizing URDF in RViz

RViz (ROS Visualization) is a powerful 3D visualizer for ROS. You will learn how to load your URDF model into RViz to inspect its structure and joint configurations.

To visualize your `simple_humanoid.urdf` in RViz:

First, ensure your `urdf_display` package is built and sourced:

```bash
# Assuming you are in your ROS 2 workspace root, e.g., ~/ros2_ws
colcon build --packages-select urdf_display
source install/setup.bash
```

Then, launch the `display_humanoid.launch.py` file:

```bash
ros2 launch urdf_display display_humanoid.launch.py
```

This will launch `robot_state_publisher`, `joint_state_publisher`, and RViz with the `simple_humanoid.urdf` loaded, using the configuration from `book-project/robotics-code/urdf-models/simple_humanoid.rviz`. You should see a simple humanoid robot model displayed in the RViz window.

## Simulating URDF in Gazebo

Gazebo is a robust 3D robotics simulator. We will demonstrate how to load your humanoid URDF into Gazebo to simulate its physical behavior. (Integration with Gazebo will be expanded upon in future modules, but the provided launch file supports it.)

## Code Examples

*   `book-project/robotics-code/urdf-models/simple_humanoid.urdf`
*   `book-project/robotics-code/urdf-models/simple_humanoid.rviz`
*   `book-project/robotics-code/ros2-examples/urdf_display/display_humanoid.launch.py`

## Further Reading

*   URDF documentation: [link to official docs]
*   RViz documentation: [link to official docs]
*   Gazebo documentation: [link to official docs]
