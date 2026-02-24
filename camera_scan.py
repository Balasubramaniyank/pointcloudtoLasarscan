import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2
import math


class PointCloudToScan(Node):

    def __init__(self):
        super().__init__('pointcloud_to_scan')

        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = 0.01
        self.range_min = 0.05
        self.range_max = 10.0

        self.subscription = self.create_subscription(
            PointCloud2,
            '/filtered_points',
            self.pointcloud_callback,
            10)

        self.publisher = self.create_publisher(
            LaserScan,
            '/camera_scan',
            10)

        self.get_logger().info("PointCloud to LaserScan Node Started")

    def pointcloud_callback(self, msg):

        scan = LaserScan()
        scan.header = msg.header
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment)
        ranges = [float('inf')] * num_ranges

        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point

            range_val = math.sqrt(x**2 + y**2)
            angle = math.atan2(y, x)

            if angle < self.angle_min or angle > self.angle_max:
                continue

            index = int((angle - self.angle_min) / self.angle_increment)

            if range_val < ranges[index]:
                ranges[index] = range_val

        scan.ranges = ranges
        self.publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
