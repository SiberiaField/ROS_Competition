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
INTERSECTION = 4


class RoadVisionNode(Node):
    def __init__(self):
        super().__init__("road_vision_node")

        self.bridge = CvBridge()

        self.state = STOP

        self.intersection_side = None

        self.max_depth = self.declare_parameter("max_depth", 0.7).value

        self.side_k = self.declare_parameter("side_k", 1.4).value

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

        self.debug_image_pub = self.create_publisher(
            Image,
            "/vision/debug_mask",
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
            self.intersection_side = self.state
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = frame.shape

        # ---------- ROI ----------
        y0 = int(h * 0.5)
        roi = frame[y0:h, :]
        depth_roi = self.depth_frame[y0:h, :]

        # --- Цветовые маски ---
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        gray_mask = cv2.inRange(
            hsv,
            np.array([0, 0, 90]),
            np.array([180, 60, 110])
        )

        kernel = np.ones((5, 5), np.uint8)
        gray_mask = cv2.morphologyEx(gray_mask, cv2.MORPH_CLOSE, kernel)
        gray_mask = cv2.morphologyEx(gray_mask, cv2.MORPH_OPEN, kernel)

        # ---------- Depth → weight ----------
        cur_max_depth = self.max_depth if self.state == STRAIGHT else self.max_depth * 0.7
        depth = depth_roi.copy()
        depth[depth == 0.0] = cur_max_depth

        depth = cv2.medianBlur(depth, 5)

        depth_norm = np.clip(depth / cur_max_depth, 0.0, 1.0)
        weight = 1.0 - depth_norm

        if self.state == INTERSECTION:
            # ---------- X-priority weight ----------
            roi_h, roi_w = weight.shape
            xs = np.linspace(-1.0, 1.0, roi_w)

            if self.intersection_side == TURN_LEFT:
                x_weight_1d = np.clip(1.0 - (xs + 1.0) / 2.0, 0.0, 1.0)
            elif self.intersection_side == TURN_RIGHT:
                x_weight_1d = np.clip((xs + 1.0) / 2.0, 0.0, 1.0)
            
            x_weight_1d = x_weight_1d * self.side_k

            # ---------- Combined weight ----------
            x_weight = np.tile(x_weight_1d, (roi_h, 1))
            weight *= x_weight

        # ---------- Road error ----------
        image_center_x = w / 2.0

        cx_gray = self.weighted_centroid_x(gray_mask, weight)

        if cx_gray is not None:
            road_center = cx_gray
        else:
            road_center = image_center_x

        road_error = (road_center - image_center_x) / image_center_x

        out = Float32()
        out.data = float(road_error)
        self.pub.publish(out)

        # ---------- DEBUG MASK ----------
        weight_img = (weight * 255).astype(np.uint8)
        final_mask = cv2.bitwise_and(gray_mask, weight_img)

        debug_full = np.zeros((h, w), dtype=np.uint8)
        debug_full[y0:h, :] = final_mask

        debug_bgr = cv2.cvtColor(debug_full, cv2.COLOR_GRAY2BGR)

        debug_msg = self.bridge.cv2_to_imgmsg(debug_bgr, encoding="bgr8")
        debug_msg.header = msg.header
        self.debug_image_pub.publish(debug_msg)

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
