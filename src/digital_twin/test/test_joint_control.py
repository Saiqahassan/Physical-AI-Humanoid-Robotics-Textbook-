import pytest
import rclpy
import time
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


@pytest.mark.rostest
def generate_test_description():
    return LaunchDescription([
        Node(
            package='digital_twin',
            executable='joint_controller.py',
            name='joint_controller',
            output='screen'
        ),
        ReadyToTest()
    ])


class TestJointControl:

    @classmethod
    def setup_class(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_joint_control_node')
        cls.publisher = cls.node.create_publisher(Twist, '/cmd_vel', 10)
        cls.joint_state_received = False
        cls.joint_state_subscriber = cls.node.create_subscription(
            JointState,
            '/joint_states',
            cls.joint_state_callback,
            10
        )

    @classmethod
    def teardown_class(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def joint_state_callback(cls, msg):
        cls.joint_state_received = True

    def test_twist_publishes_joint_state(self):
        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        self.publisher.publish(twist_msg)

        # Wait for joint state message to be received
        timeout_sec = 5
        start_time = time.time()
        while not self.joint_state_received and \
                (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
                
        
                assert self.joint_state_received, \
        
                    "Did not receive JointState message after publishing Twist"
        
        
