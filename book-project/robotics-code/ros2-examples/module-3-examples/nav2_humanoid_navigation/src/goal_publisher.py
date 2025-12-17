# goal_publisher.py
# This script is a placeholder for a ROS 2 node/script that sends navigation goals to Nav2.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator # Requires nav2_simple_commander package

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.get_logger().info('GoalPublisher node has been started.')

        self.navigator = BasicNavigator()

        # Wait for Nav2 to be active
        self.navigator.waitForPose()
        self.get_logger().info('Nav2 is ready.')

        # Example: Define a goal pose
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = 2.0
        self.goal_pose.pose.position.y = 0.5
        self.goal_pose.pose.orientation.w = 1.0 # Facing forward

        # TODO: Publish the goal
        # self.navigator.goToPose(self.goal_pose)
        # self.get_logger().info(f"Sending goal: {self.goal_pose.pose.position.x}, {self.goal_pose.pose.position.y}")

        # You can add logic to wait for completion or send multiple goals
        # while not self.navigator.isTaskComplete():
        #     feedback = self.navigator.getFeedback()
        #     if feedback and feedback.navigation_time > 600.0: # Timeout after 10 mins
        #         self.navigator.cancelTask()
        #     # print(f"Distance remaining: {feedback.distance_remaining}")

        # result = self.navigator.getResult()
        # if result == TaskResult.SUCCEEDED:
        #     self.get_logger().info('Goal succeeded!')
        # else:
        #     self.get_logger().warn('Goal failed or was cancelled!')


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    # No rclpy.spin() needed as BasicNavigator is blocking, or you can add a timer for non-blocking
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
