import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np


class HeightFilter(Node):

    def __init__(self):
        super().__init__('height_filter_node')

        # Height limits for small obstacles
        self.min_height = 0.02
        self.max_height = 0.20

        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.pointcloud_callback,
            10)

        self.publisher = self.create_publisher(
            PointCloud2,
            '/filtered_points',
            10)

        self.get_logger().info("Height Filter Node Started")

    def pointcloud_callback(self, msg):

        filtered_points = []

        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point

            if self.min_height < z < self.max_height:
                filtered_points.append([x, y, z])

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id

        filtered_cloud = pc2.create_cloud_xyz32(header, filtered_points)
        self.publisher.publish(filtered_cloud)


def main(args=None):
    rclpy.init(args=args)
    node = HeightFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
