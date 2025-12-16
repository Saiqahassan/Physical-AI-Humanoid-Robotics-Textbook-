# ROS 2 as the Robotic Nervous System

This chapter introduces ROS 2 (Robot Operating System 2) as the fundamental middleware for building robotic applications. It will cover the core concepts of ROS 2, explaining how it acts as the "nervous system" for robots, enabling different components to communicate and work together.

## Core Concepts

*   **Nodes**: Independent processes that perform computation.
*   **Topics**: The primary mechanism for asynchronous, one-way data streaming between nodes.
*   **Services**: Request/reply communication patterns for synchronous, two-way interactions.
*   **Parameters**: Dynamic configuration values for nodes.
*   **Actions**: For long-running, goal-oriented tasks with feedback and cancellation capabilities.

## Setting up Your ROS 2 Environment

This section will provide detailed instructions on setting up a ROS 2 environment on Ubuntu 22.04 with ROS 2 Humble/Iron. It will include:
*   Installation steps.
*   Environment configuration.
*   Verification of installation.

## Running a Minimal ROS 2 Node Graph

Here, we will demonstrate how to run a basic ROS 2 node graph using simple `talker` and `listener` examples. This will illustrate topic-based communication in action.

First, navigate to your ROS 2 workspace (or a directory where you'd like to create one) and clone the `robotics-code` repository or copy the `minimal_nodes` package.

```bash
# Example: If you cloned the entire book-project
cd ~/book-project/robotics-code/ros2-examples/minimal_nodes
```

Then, build the package using `colcon`:

```bash
# Assuming you are in your ROS 2 workspace root, e.g., ~/ros2_ws
colcon build --packages-select minimal_nodes
source install/setup.bash
```

Now, you can run the `talker` node in one terminal:

```bash
ros2 run minimal_nodes talker
```

And the `listener` node in another terminal:

```bash
ros2 run minimal_nodes listener
```

You should see the `talker` publishing "Hello World" messages and the `listener` receiving and printing them to its console.

**(Diagram for ROS 2 node graph will be integrated here later)**

## Code Examples

You can find the full code for these examples in:
*   `book-project/robotics-code/ros2-examples/minimal_nodes/talker.py`
*   `book-project/robotics-code/ros2-examples/minimal_nodes/listener.py`

## Further Reading

*   ROS 2 Documentation: [link to official docs]
*   `rclpy` documentation: [link to official docs]
