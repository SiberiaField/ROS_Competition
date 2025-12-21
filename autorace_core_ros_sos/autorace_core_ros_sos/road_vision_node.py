import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32

from cv_bridge import CvBridge
import cv2
import numpy as np


STRAIGHT = 0
IN_INTERSECTION = 1
EXIT = 2


class RoadVisionNode(Node):
    def __init__(self):
        super().__init__('road_vision_node')

        self.bridge = CvBridge()

        self.state = STRAIGHT
        self.turn_direction = self.declare_parameter(
            'turn_direction', 'right'
        ).value

        self.create_subscription(
            Image,
            '/color/image',
            self.image_callback,
            10
        )

        self.create_subscription(
            Int32,
            '/fsm_state',
            self.state_callback,
            10
        )

        self.pub = self.create_publisher(
            Float32,
            '/road_error',
            10
        )

        self.get_logger().info(
            f'Vision node started, turn = {self.turn_direction}'
        )

    def state_callback(self, msg):
        self.state = msg.data

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, _ = frame.shape

        roi = frame[int(h * 0.5):h, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # --- Цветовые маски ---
        white_mask = cv2.inRange(
            hsv,
            np.array([0, 0, 200]),
            np.array([180, 30, 255])
        )

        yellow_mask = cv2.inRange(
            hsv,
            np.array([15, 80, 80]),
            np.array([35, 255, 255])
        )

        kernel = np.ones((5, 5), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

        image_center_x = w / 2.0

        # ===== STRAIGHT / EXIT =====
        if self.state in [STRAIGHT, EXIT]:
            cx_white = self.centroid_x(white_mask)
            cx_yellow = self.centroid_x(yellow_mask)

            if cx_white is not None and cx_yellow is not None:
                road_center = (cx_white + cx_yellow) / 2.0
            elif cx_white is not None:
                road_center = cx_white
            elif cx_yellow is not None:
                road_center = cx_yellow
            else:
                road_center = image_center_x

        # ===== IN_INTERSECTION =====
        else:
            if self.turn_direction == 'right':
                ref_mask = yellow_mask
                offset = -80   # смещение влево от жёлтой
            else:
                ref_mask = white_mask
                offset = 80    # смещение вправо от белой

            ref_point = self.forward_point(ref_mask)

            if ref_point is not None:
                road_center = ref_point[0] + offset
            else:
                road_center = image_center_x

        road_error = (road_center - image_center_x) / image_center_x

        out = Float32()
        out.data = float(road_error)
        self.pub.publish(out)

    # ---------- helpers ----------

    def centroid_x(self, mask):
        m = cv2.moments(mask)
        if m['m00'] > 0:
            return int(m['m10'] / m['m00'])
        return None

    def forward_point(self, mask):
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        point = min(largest, key=lambda p: p[0][1])
        return point[0]  # (x, y)


def main(args=None):
    rclpy.init(args=args)
    node = RoadVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
