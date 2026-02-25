import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
import math


class PointToScan(Node):

    def __init__(self):
        super().__init__('point_to_scan_node')

        self.subscription = self.create_subscription(
            PointStamped,
            '/detected_obstacle',
            self.point_callback,
            10)

        self.publisher = self.create_publisher(
            LaserScan,
            '/camera_scan',
            10)

        # Laser parameters
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = 0.01
        self.range_min = 0.05
        self.range_max = 10.0

        self.get_logger().info("Point to Scan Node Started")

    def point_callback(self, msg):

        x = msg.point.x
        y = msg.point.y

        distance = math.sqrt(x**2 + y**2)
        angle = math.atan2(y, x)

        scan = LaserScan()
        scan.header.frame_id = "map"
        scan.header.stamp = self.get_clock().now().to_msg()

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        size = int((self.angle_max - self.angle_min) / self.angle_increment)
        ranges = [float('inf')] * size

        index = int((angle - self.angle_min) / self.angle_increment)

        if 0 <= index < size:
            ranges[index] = distance

        scan.ranges = ranges

        self.publisher.publish(scan)
