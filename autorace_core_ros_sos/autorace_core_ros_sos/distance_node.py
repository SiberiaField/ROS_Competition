import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

import math
import numpy as np


class DistanceNode(Node):
    def __init__(self):
        super().__init__("distance_node")

        self.angle_view = self.declare_parameter("lidar_angle_view", 20.0).value   # +/- градусов впереди
        self.max_distance = self.declare_parameter("max_lidar_dist", 5.0).value         # максимальная дистанция (м)
        self.min_distance = self.declare_parameter("min_lidar_dist", 0.05).value         # минимальная (шум)

        self.sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        self.front_pub = self.create_publisher(
            Float32,
            "/lidar/front_distance",
            10
        )

        self.back_pub = self.create_publisher(
            Float32,
            "/lidar/back_distance",
            10
        )

        self.get_logger().info("Front distance lidar node started")

    def scan_callback(self, msg: LaserScan):
        back_angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        front_angles = back_angles + math.pi
        front_angles = (front_angles + math.pi) % (2 * math.pi) - math.pi

        ranges = np.array(msg.ranges)

        front_mask = np.abs(front_angles) < math.radians(self.angle_view)
        back_mask = np.abs(back_angles) < math.radians(self.angle_view)

        front_ranges = ranges[front_mask]
        front_ranges = front_ranges[np.isfinite(front_ranges)]
        front_ranges = front_ranges[
            (front_ranges > self.min_distance) &
            (front_ranges < self.max_distance)
        ]

        back_ranges = ranges[back_mask]
        back_ranges = back_ranges[np.isfinite(back_ranges)]
        back_ranges = back_ranges[
            (back_ranges > self.min_distance) &
            (back_ranges < self.max_distance)
        ]

        if len(front_ranges) == 0:
            front_distance = self.max_distance
        else:
            front_distance = float(np.min(front_ranges))

        if len(back_ranges) == 0:
            back_distance = 0.0
        else:
            back_distance = float(np.min(back_ranges))

        msg_out = Float32()
        msg_out.data = front_distance
        self.front_pub.publish(msg_out)
        msg_out.data = back_distance
        self.back_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = DistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
