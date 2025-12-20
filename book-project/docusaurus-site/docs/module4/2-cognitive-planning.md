# Chapter 2: Language-Based Cognitive Planning and Task Decomposition

This chapter delves into how Large Language Models (LLMs) can be leveraged for cognitive planning and task decomposition in robotics. Moving beyond simple, direct commands, we explore how an LLM can transform high-level natural language goals into structured, executable action plans for a humanoid robot.

## The Role of LLMs in Robotics Planning

Traditional robotics planning often relies on predefined state machines or symbolic planners that require explicit models of the environment and actions. LLMs, with their vast knowledge and reasoning capabilities, offer a more flexible approach:

-   **Natural Language Understanding**: LLMs can interpret complex human instructions, handling ambiguities and context that are challenging for rule-based systems.
-   **Task Decomposition**: They can break down a high-level goal into a sequence of smaller, actionable steps.
-   **Common Sense Reasoning**: LLMs can incorporate common-sense knowledge to infer implicit details or constraints in a command.

## Implementing the LLM Planner Node

Our `llm_planner_node.py` serves as the brain of our cognitive planning system. It receives transcribed text commands and, using the GPT-4 API, generates a JSON-formatted action plan.

### `llm_planner_node.py` Code

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import os
import json

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')
        self.subscription = self.create_subscription(
            String,
            'transcribed_text',
            self.listener_callback,
            10)
        self.action_plan_publisher = self.create_publisher(String, 'action_plan', 10)
        openai.api_key = os.getenv("OPENAI_API_KEY")
        self.get_logger().info('LLM planner node started')

    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: "{command}"')

        try:
            prompt = f"""
            You are a robotics planner. Convert the following natural language command into a structured JSON action plan.
            The available actions are: "wave_hello()", "walk_forward()", "navigate_to(destination)", "find_object(object_name)", "pick_up(object_id)".
            
            Command: "{command}"

            Return only the JSON object.
            """

            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": "You are a robotics planner."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.0,
            )

            plan_text = response.choices[0].message['content']
            self.get_logger().info(f"Generated plan: {plan_text}")

            json.loads(plan_text) # basic validation
            plan_msg = String()
            plan_msg.data = plan_text
            self.action_plan_publisher.publish(plan_msg)

        except Exception as e:
            self.get_logger().error(f"Error during planning: {e}")

# ... (main function)
```

### Robot Controller Updates

The `robot_controller_node.py` is updated to subscribe to the `action_plan` topic from the LLM planner. It now parses the JSON plan and executes each action sequentially.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
import time
import json

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        self.subscription = self.create_subscription(
            String,
            'action_plan', # Subscribing to action_plan topic
            self.listener_callback,
            10)
        
        # ... (joint_position_publisher and init log)

    def listener_callback(self, msg):
        plan_str = msg.data
        self.get_logger().info(f'Received action plan: "{plan_str}"')

        try:
            plan = json.loads(plan_str)
            tasks = plan.get('tasks', [])
            
            for task in tasks:
                action_name = task.get('action_name')
                parameters = task.get('parameters', {})
                self.execute_task(action_name, parameters)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse action plan: {e}")

    def execute_task(self, action_name, parameters):
        if action_name == "wave_hello":
            self.wave_hello()
        elif action_name == "walk_forward":
            self.walk_forward()
        # ... (other actions)
        else:
            self.get_logger().warn(f"Unknown action: {action_name}")

    # ... (wave_hello, walk_forward methods and main function)
```

### Launch File Updates

The `voice_control.launch.py` file is updated to include the new `llm_planner_node`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vla_core',
            executable='speech_to_text_node',
            name='speech_to_text_node',
            output='screen'
        ),
        Node(
            package='vla_core',
            executable='llm_planner_node', # New node added
            name='llm_planner_node',
            output='screen'
        ),
        Node(
            package='vla_core',
            executable='robot_controller_node',
            name='robot_controller_node',
            output='screen'
        ),
    ])
```

By completing this chapter, you will have equipped your robot with the ability to interpret and plan for more complex, high-level commands, moving closer to truly autonomous behavior.
