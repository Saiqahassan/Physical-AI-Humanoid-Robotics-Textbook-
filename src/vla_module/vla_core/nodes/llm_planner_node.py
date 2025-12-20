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
            self.transcribed_text_callback,
            10)
        self.perception_subscription = self.create_subscription(
            String,
            'detected_objects',
            self.detected_objects_callback,
            10)
        self.action_plan_publisher = self.create_publisher(String, 'action_plan', 10)
        openai.api_key = os.getenv("OPENAI_API_KEY")
        self.get_logger().info('LLM planner node started')
        self.detected_objects = []

    def detected_objects_callback(self, msg):
        try:
            self.detected_objects = json.loads(msg.data)
            self.get_logger().info(f"Received detected objects: {self.detected_objects}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse detected objects: {e}")

    def transcribed_text_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: "{command}"')

        try:
            # Prepare detected objects for the prompt
            objects_info = ""
            if self.detected_objects:
                objects_info = "Currently detected objects in view:\n"
                for obj in self.detected_objects:
                    objects_info += f"- {obj['label']} (ID: {obj['object_id']}) at estimated camera-relative position (x:{obj['pose']['x']:.2f}, y:{obj['pose']['y']:.2f}, z:{obj['pose']['z']:.2f})\n"
            
            # Create a structured prompt for the LLM
            prompt = f"""
            You are a robotics planner. Convert the following natural language command into a structured JSON action plan.
            The available actions are: "wave_hello()", "walk_forward()", "navigate_to(destination)", "find_object(object_name)", "pick_up(object_id)".

            {objects_info}
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

            # Validate and publish the plan
            # A real implementation should have robust validation
            json.loads(plan_text) # basic validation
            plan_msg = String()
            plan_msg.data = plan_text
            self.action_plan_publisher.publish(plan_msg)

        except Exception as e:
            self.get_logger().error(f"Error during planning: {e}")


def main(args=None):
    rclpy.init(args=args)
    llm_planner_node = LLMPlannerNode()
    rclpy.spin(llm_planner_node)
    llm_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
