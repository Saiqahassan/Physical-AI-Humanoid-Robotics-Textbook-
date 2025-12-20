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
            'action_plan',
            self.listener_callback,
            10)
        
        # Publishers for joint positions
        self.joint_position_publisher = self.create_publisher(
            Float64MultiArray, 
            '/humanoid_joint_controller/commands', 
            10
        )

        self.get_logger().info('Robot controller node started')

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
        elif action_name == "pick_up":
            object_id = parameters.get('object_id')
            self.pick_up(object_id)
        else:
            self.get_logger().warn(f"Unknown action: {action_name}")

    def pick_up(self, object_id):
        self.get_logger().info(f"Executing: pick up {object_id} (placeholder)")
        # A real implementation would involve:
        # 1. Inverse kinematics to move the arm to the object's position.
        # 2. Gripper control to grasp the object.
        # 3. Attaching the object to the gripper in the simulation (e.g., using Gazebo physics API).
        # 4. Lifting the object.

    def wave_hello(self):
        self.get_logger().info("Executing: wave hello")
        # This is a simplified example. A real implementation would use MoveIt! or a more
        # sophisticated trajectory controller.
        # This assumes a joint controller that accepts position commands for specific joints.
        # The Float64MultiArray would contain positions for right_arm_joint, etc.
        
        # Raise arm
        msg = Float64MultiArray()
        # right_arm_joint index 4, angle -1.57
        msg.data = [0.0, 0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0] 
        self.joint_position_publisher.publish(msg)
        time.sleep(1)

        # Wave
        msg.data = [0.0, 0.0, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0]
        self.joint_position_publisher.publish(msg)
        time.sleep(0.5)
        msg.data = [0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0]
        self.joint_position_publisher.publish(msg)
        time.sleep(0.5)

        # Lower arm
        msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_position_publisher.publish(msg)


    def walk_forward(self):
        # A real walking implementation is extremely complex and would involve
        # a dedicated walking controller or MPC. This is a placeholder.
        self.get_logger().info("Executing: walk forward (placeholder)")
        # Example: publish to cmd_vel if the robot was wheeled
        # from geometry_msgs.msg import Twist
        # twist = Twist()
        # twist.linear.x = 0.5
        # self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    robot_controller_node = RobotControllerNode()
    rclpy.spin(robot_controller_node)
    robot_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
