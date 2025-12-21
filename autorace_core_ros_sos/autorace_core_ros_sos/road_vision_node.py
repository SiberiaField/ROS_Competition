import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32

from cv_bridge import CvBridge
import cv2
import numpy as np


STOP = 0
STRAIGHT = 1
TURN_RIGHT = 2
TURN_LEFT = 3


class RoadVisionNode(Node):
    def __init__(self):
        super().__init__("road_vision_node")

        self.bridge = CvBridge()

        self.state = STOP
        self.turn_direction = self.declare_parameter(
            "turn_direction", "right"
        ).value

        self.create_subscription(
            Image,
            "/color/image",
            self.image_callback,
            10
        )

        self.create_subscription(
            Image,
            "/depth/image",
            self.depth_callback,
            10
        )

        self.create_subscription(
            Int32,
            "/fsm_state",
            self.state_callback,
            10
        )

        self.motion_fsm_state_pub = self.create_publisher(
            Int32,
            "/motion_fsm_state",
            1
        )

        self.pub = self.create_publisher(
            Float32,
            "/road_error",
            1
        )

        self.get_logger().info(
            "Vision node started"
        )

    def state_callback(self, msg):
        self.state = msg.data
        self.motion_fsm_state_pub.publish(msg)

    def depth_callback(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg)

        # depth может быть 16UC1 или 32FC1
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) / 1000.0  # мм → м

        self.depth_frame = depth

    def image_callback(self, msg):
        if self.state == STOP or self.depth_frame is None:
            return
        elif self.state in [TURN_RIGHT, TURN_LEFT]:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = frame.shape

        # ---------- ROI ----------
        y0 = int(h * 0.5)
        roi = frame[y0:h, :]
        depth_roi = self.depth_frame[y0:h, :]

        # --- Цветовые маски ---
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

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

        # ---------- Depth → weight ----------
        max_depth = 5.0 # метры
        depth = depth_roi.copy()
        depth[depth == 0.0] = max_depth

        depth = cv2.medianBlur(depth, 5)

        depth_norm = np.clip(depth / max_depth, 0.0, 1.0)
        weight = 1.0 - depth_norm   # ближе = больше вес

        image_center_x = w / 2.0

        cx_white = self.weighted_centroid_x(white_mask, weight)
        cx_yellow = self.weighted_centroid_x(yellow_mask, weight)

        if cx_white is not None and cx_yellow is not None:
            road_center = (cx_white + cx_yellow) / 2.0
        elif cx_white is not None:
            road_center = cx_white
        elif cx_yellow is not None:
            road_center = cx_yellow
        else:
            road_center = image_center_x

        road_error = (road_center - image_center_x) / image_center_x

        out = Float32()
        out.data = float(road_error)
        self.pub.publish(out)

    # ---------- helpers ----------

    def weighted_centroid_x(self, mask, weight):
        mask_norm = (mask > 0).astype(np.float32)
        weighted = mask_norm * weight

        ys, xs = np.where(weighted > 0)

        if len(xs) < 50:
            return None

        w_sum = np.sum(weighted[ys, xs])
        cx = np.sum(xs * weighted[ys, xs]) / w_sum
        return cx


def main(args=None):
    rclpy.init(args=args)
    node = RoadVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
