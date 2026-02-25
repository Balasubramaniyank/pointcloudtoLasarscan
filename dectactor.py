import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageDetector(Node):

    def __init__(self):
        super().__init__('image_detector')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.get_logger().info("OpenCV Detector Started")

    def image_callback(self, msg):

        # Convert ROS image to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define red color range
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])

        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area > 500:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)

                center_x = x + w//2
                center_y = y + h//2

                self.get_logger().info(f"Object detected at pixel: {center_x}, {center_y}")

        cv2.imshow("Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
