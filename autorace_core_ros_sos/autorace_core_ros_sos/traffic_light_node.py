import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from cv_bridge import CvBridge
import cv2
import numpy as np


class TrafficLightNode(Node):
    def __init__(self):
        super().__init__("traffic_light_node")

        self.bridge = CvBridge()

        self.green_threshold = self.declare_parameter("green_threshold", 500).value

        self.create_subscription(
            Image,
            "/color/image",
            self.image_callback,
            10
        )

        self.fsm_state_pub = self.create_publisher(
            Int32,
            "/fsm_state",
            1
        )

        self.get_logger().info("Traffic light node started")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        green_mask = cv2.inRange(
            hsv,
            np.array([35, 80, 80]),
            np.array([85, 255, 255])
        )

        kernel = np.ones((5, 5), np.uint8)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

        green_pixels = cv2.countNonZero(green_mask)

        if green_pixels > self.green_threshold:
            fsm_state = Int32()
            fsm_state.data = 1
            self.fsm_state_pub.publish(fsm_state)
            rclpy.spin_once(self, timeout_sec=1.0)
            self.get_logger().info("Traffic light node shutdown")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
