import time
from math import pi

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist


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


STOP = 0
STRAIGHT = 1
TURN_RIGHT = 2
TURN_LEFT = 3


class MotionController(Node):
    def __init__(self):
        super().__init__("motion_controller")

        # FSM
        self.cur_state = STOP
        self.active_state = STOP
        self.last_state_change = time.time()

        # Error from vision
        self.road_error = 0.0

        # PID
        self.pid = PID(
            kp=self.declare_parameter("kp", 1.0).value,
            ki=self.declare_parameter("ki", 0.0).value,
            kd=self.declare_parameter("kd", 0.001).value,
            output_limit=self.declare_parameter("error_limit", 1.5).value
        )

        self.last_time = self.get_clock().now()

        # ROS interfaces
        self.create_subscription(
            Float32,
            "/road_error",
            self.error_callback,
            10
        )

        self.create_subscription(
            Int32,
            "/motion_fsm_state",
            self.state_callback,
            10
        )

        self.fsm_state_pub = self.create_publisher(
            Int32,
            "/fsm_state",
            1
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            1
        )

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("Motion controller started")

    def error_callback(self, msg):
        self.road_error = msg.data

    def state_callback(self, msg):
        self.cur_state = msg.data

    def control_loop(self):
        if self.active_state == STOP and (self.cur_state == STOP):
            return
        elif self.active_state == STOP and (self.cur_state != STOP):
            self.active_state = self.cur_state

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        cmd = Twist()

        # -------- FSM --------
        if self.active_state == STRAIGHT:
            cmd.linear.x = 0.2
            if self.cur_state in [TURN_RIGHT, TURN_LEFT]:
                self.pid.reset()
                self.active_state = self.cur_state
            else:
                angular = self.pid.compute(self.road_error, dt)
                cmd.angular.z = -angular
                self.cmd_pub.publish(cmd)
        elif self.active_state in [TURN_RIGHT, TURN_LEFT]:
            cmd.linear.x = 0.05
            cmd.angular.z = 4*pi if self.active_state == TURN_LEFT else -4*pi

            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=5.0)

            fsm_state = Int32()
            fsm_state.data = 1
            self.active_state = self.cur_state = STRAIGHT
            self.fsm_state_pub.publish(fsm_state)
            self.get_logger().info("Completed turning")


def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
