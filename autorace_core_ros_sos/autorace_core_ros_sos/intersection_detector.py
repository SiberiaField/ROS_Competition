import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Bool, Float32

from cv_bridge import CvBridge
import cv2
import numpy as np


STOP = 0
STRAIGHT = 1
TURN_RIGHT = 2
TURN_LEFT = 3
INTERSECTION = 4


class IntersectionDetector(Node):
    def __init__(self):
        super().__init__("intersection_detector")

        self.bridge = CvBridge()

        self.new_state = None
        self.stop_working = False

        self.dist_threshold = self.declare_parameter("front_dist_threshold", 1.0).value
        self.front_dist = None

        self.create_subscription(
            Image,
            "/color/image",
            self.image_callback,
            10
        )

        self.create_subscription(
            Bool,
            "/stop_intersection_node",
            self.stop_intersection_node_callback,
            10
        )

        self.create_subscription(
            Float32,
            "/lidar/front_distance",
            self.front_dist_callback,
            10
        )

        self.fsm_state_pub = self.create_publisher(
            Int32,
            "/fsm_state",
            1
        )

        self.debug_pub = self.create_publisher(
            Image,
            "/turn_sign/debug_image",
            1
        )

        self.get_logger().info("Intersection detector started")

    def stop_intersection_node_callback(self, msg):
        self.stop_working = msg.data

    def front_dist_callback(self, msg):
        self.front_dist = msg.data

    def image_callback(self, msg):
        if self.stop_working is True:
            rclpy.shutdown()

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        h, w, _ = frame.shape
        frame = frame[0:int(h * 0.6), :]   # верх кадра не нужен

        debug = frame.copy()

        # =====================================================
        # 1. Поиск синего знака (HSV)
        # =====================================================
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([140, 255, 255])

        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        blue_mask = cv2.morphologyEx(
            blue_mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)
        )

        # ===== DEBUG: синяя маска =====
        blue_overlay = debug.copy()
        blue_overlay[blue_mask > 0] = (255, 0, 0)
        debug = cv2.addWeighted(debug, 0.7, blue_overlay, 0.3, 0)
        # ==============================

        contours, _ = cv2.findContours(
            blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        contours = [c for c in contours if cv2.contourArea(c) > 2000]
        if not contours:
            self.publish_debug(debug)
            return

        # =====================================================
        # 2. ROI знака
        # =====================================================
        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)

        if w < 40 or h < 40:
            self.publish_debug(debug)
            return

        roi = frame[y:y+h, x:x+w]
        roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        cv2.rectangle(debug, (x, y), (x+w, y+h), (0, 255, 255), 2)

        # =====================================================
        # 3. Белая стрелка
        # =====================================================
        blue_roi_mask = cv2.inRange(
            roi_hsv,
            lower_blue,
            upper_blue
        )

        arrow_mask = cv2.bitwise_not(blue_roi_mask)

        # чистим шум
        arrow_mask = cv2.morphologyEx(
            arrow_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
        )

        # Arrow ROI
        h_roi, w_roi = arrow_mask.shape

        top = int(0.25 * h_roi)
        bottom = int(0.35 * h_roi)
        left = int(0.10 * w_roi)
        right = int(0.10 * w_roi)

        arrow_mask[:top, :] = 0
        arrow_mask[h_roi - bottom:, :] = 0
        arrow_mask[:, :left] = 0
        arrow_mask[:, w_roi - right:] = 0

        arrow_vis = cv2.cvtColor(arrow_mask, cv2.COLOR_GRAY2BGR)
        debug[y:y+h, x:x+w] = cv2.addWeighted(
            debug[y:y+h, x:x+w],
            0.6,
            arrow_vis,
            0.4,
            0
        )

        contours, _ = cv2.findContours(
            arrow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for c in contours:
            cv2.drawContours(
                debug[y:y+h, x:x+w],
                [c],
                -1,
                (0, 0, 255),
                2
            )

        arrow = max(contours, key=cv2.contourArea)

        # =====================================================
        # 4. Геометрия стрелки
        # =====================================================
        M = cv2.moments(arrow)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        pts = arrow.reshape(-1, 2)

        # минимальный y = самая верхняя точка
        top_idx = np.argmin(pts[:, 1])
        top_pt = pts[top_idx]

        # =====================================================
        # 5. Направление
        # =====================================================
        dx = top_pt[0] - cx

        label = "UNKNOWN"

        if dx > 0:
            label = "RIGHT"
            self.new_state = TURN_RIGHT
        elif dx < 0:
            label = "LEFT"
            self.new_state = TURN_LEFT

        # =====================================================
        # 6. Debug визуализация
        # =====================================================
        cv2.circle(
            debug,
            (x + cx, y + cy),
            5,
            (0, 255, 0),
            -1
        )

        cv2.circle(
            debug,
            (x + top_pt[0], y + top_pt[1]),
            7,
            (0, 0, 255),
            -1
        )

        cv2.line(
            debug,
            (x + cx, y + cy),
            (x + top_pt[0], y + top_pt[1]),
            (255, 255, 0),
            2
        )

        cv2.putText(
            debug,
            f"SIGN: {label}",
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2
        )

        # =====================================================
        # 7. Публикация
        # =====================================================

        self.publish_debug(debug)
        if self.new_state is not None:
            if self.front_dist < self.dist_threshold:
                fsm_state = Int32()
                fsm_state.data = self.new_state
                self.fsm_state_pub.publish(fsm_state)

    def publish_debug(self, img):
        debug_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IntersectionDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
