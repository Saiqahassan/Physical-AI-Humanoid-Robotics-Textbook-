# Python Agents + ROS 2 (rclpy)

This chapter dives deeper into ROS 2 communication by focusing on `rclpy`, the Python client library for ROS 2. You will learn how to develop custom ROS 2 nodes using Python to create publishers and subscribers, enabling your robot's components to exchange data efficiently.

## Understanding `rclpy`

`rclpy` provides Python bindings for the ROS Client Library (RCL), allowing developers to interact with the ROS 2 graph using Python. It offers a straightforward API for creating nodes, publishers, subscribers, services, and actions.

## Implementing Publishers

Publishers are responsible for sending data (messages) on a specific topic. We will walk through creating a Python node that publishes custom messages.

To run the publisher example, first ensure your `pubsub_rclpy` package is built and sourced:

```bash
# Assuming you are in your ROS 2 workspace root, e.g., ~/ros2_ws
colcon build --packages-select pubsub_rclpy
source install/setup.bash
```

Then, run the `simple_publisher` node in one terminal:

```bash
ros2 run pubsub_rclpy simple_publisher
```

You should see it publishing messages to the `custom_topic`.

## Implementing Subscribers

Subscribers are responsible for receiving data from a specific topic. We will demonstrate how to create a Python node that subscribes to a topic and processes incoming messages.

In another terminal (after building and sourcing as above), run the `simple_subscriber` node:

```bash
ros2 run pubsub_rclpy simple_subscriber
```

You should see the `simple_subscriber` receiving and printing the messages published by the `simple_publisher`.

## Code Examples

You can find the full code for these examples in:
*   `book-project/robotics-code/ros2-examples/pubsub_rclpy/simple_publisher.py`
*   `book-project/robotics-code/ros2-examples/pubsub_rclpy/simple_subscriber.py`

## Further Reading

*   `rclpy` API Documentation: [link to official docs]
*   ROS 2 Python Tutorials: [link to official tutorials]
