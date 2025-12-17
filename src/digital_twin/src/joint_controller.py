import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class JointController(Node):

    def __init__(self):
        super().__init__('joint_controller')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(
            JointState, '/joint_states', 10)

    def listener_callback(self, msg):
        self.get_logger().info('Received Twist message: "%s"' % msg)
        # In a real controller, you would use the Twist message
        # to calculate the desired joint positions and publish them.
        # For now, we'll just publish a static joint state.
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            'left_shoulder_joint', 'right_shoulder_joint',
            'left_hip_joint', 'right_hip_joint'
        ]
        joint_state.position = [0.0, 0.0, 0.0, 0.0]
        self.publisher_.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    joint_controller = JointController()
    rclpy.spin(joint_controller)
    joint_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
