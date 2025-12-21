import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
import time


class PID:
    def __init__(self, kp, ki, kd, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = output_limit

        self.prev_error = 0.0
        self.integral = 0.0

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0.0 else 0.0

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.prev_error = error
        return max(-self.limit, min(self.limit, output))


STRAIGHT = 0
IN_INTERSECTION = 1
EXIT = 2


class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')

        # FSM
        self.state = STRAIGHT
        self.last_state_change = time.time()

        # Error from vision
        self.road_error = 0.0
        self.intersection_detected = False

        # PID
        self.pid = PID(
            kp=self.declare_parameter('kp', 2.5).value,
            ki=self.declare_parameter('ki', 0.0).value,
            kd=self.declare_parameter('kd', 0.0).value,
            output_limit=self.declare_parameter('error_limit', 1.5).value
        )

        self.last_time = self.get_clock().now()

        # ROS interfaces
        self.create_subscription(
            Float32,
            '/road_error',
            self.error_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/intersection',
            self.intersection_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("Motion controller started")

    def error_callback(self, msg):
        self.road_error = msg.data

    def intersection_callback(self, msg):
        self.intersection_detected = msg.data

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        cmd = Twist()

        # -------- FSM --------
        if self.state == STRAIGHT:
            cmd.linear.x = 0.2

            if self.intersection_detected:
                self.state = IN_INTERSECTION
                self.pid.reset()
                self.last_state_change = time.time()

        elif self.state == IN_INTERSECTION:
            cmd.linear.x = 0.2

            # фиктивное условие выхода (по времени)
            if time.time() - self.last_state_change > 3.0:
                self.state = EXIT
                self.pid.reset()
                self.last_state_change = time.time()

        elif self.state == EXIT:
            cmd.linear.x = 0.2

            if time.time() - self.last_state_change > 1.5:
                self.state = STRAIGHT
                self.pid.reset()

        # -------- PID --------
        angular = self.pid.compute(self.road_error, dt)
        cmd.angular.z = -angular   # минус — стандартно для изображения

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
