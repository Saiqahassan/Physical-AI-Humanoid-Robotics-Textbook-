import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Assuming camera publishes to this topic
            self.image_callback,
            10)
        self.publisher = self.create_publisher(String, 'detected_objects', 10)
        self.bridge = CvBridge()
        self.get_logger().info('Perception node started, looking for red blocks.')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        # Convert BGR to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define range for red color in HSV
        # This range might need tuning depending on the lighting in Gazebo
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = mask1 + mask2

        # Find contours in the mask
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_objects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Filter small areas
                x, y, w, h = cv2.boundingRect(contour)
                
                # For simplicity, we'll assume a fixed depth and estimate 3D position
                # In a real scenario, depth sensing (e.g., from a depth camera) would be used
                # Also, transformation from camera frame to world frame would be needed
                estimated_depth = 1.0 # Placeholder depth in meters
                
                # Estimate object's position relative to camera (simplified)
                # This is a very rough estimate and would need calibration and depth data
                object_x_cam = (x + w/2 - cv_image.shape[1]/2) * 0.001 * estimated_depth # Rough pixel to meter conversion
                object_y_cam = (y + h/2 - cv_image.shape[0]/2) * 0.001 * estimated_depth # Rough pixel to meter conversion
                
                object_info = {
                    "object_id": f"red_block_{len(detected_objects)}",
                    "label": "red block",
                    "pose": {
                        "x": estimated_depth, # Assuming depth is forward
                        "y": -object_x_cam, # Invert x for camera to robot frame
                        "z": -object_y_cam # Invert y for camera to robot frame
                    },
                    "bounding_box": {"x": x, "y": y, "w": w, "h": h}
                }
                detected_objects.append(object_info)
                self.get_logger().info(f"Detected: {object_info['label']} at estimated position ({object_info['pose']['x']:.2f}, {object_info['pose']['y']:.2f}, {object_info['pose']['z']:.2f})")

        if detected_objects:
            detection_msg = String()
            detection_msg.data = json.dumps(detected_objects)
            self.publisher.publish(detection_msg)


def main(args=None):
    rclpy.init(args=args)
    perception_node = PerceptionNode()
    rclpy.spin(perception_node)
    perception_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
