import time
from math import pi

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import tf_transformations


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
INTERSECTION = 4


class MotionController(Node):
    def __init__(self):
        super().__init__("motion_controller")

        # FSM
        self.active_state = STOP
        self.cur_state = STOP
        self.last_state_change = time.time()

        # Yaw
        self.current_yaw = None
        self.yaw_threshold = self.declare_parameter("yaw_threshold", 5.0).value
        self.yaw_threshold = self.yaw_threshold * (pi / 180.0)

        # Error from vision
        self.road_error = 0.0

        # PID
        self.center_pid = PID(
            kp=self.declare_parameter("c_p", 3.0).value,
            ki=self.declare_parameter("c_i", 0.0).value,
            kd=self.declare_parameter("c_d", 0.0).value,
            output_limit=self.declare_parameter("c_error_limit", 5.0).value
        )

        self.turn_pid = PID(
            kp=self.declare_parameter("t_p", 0.8).value,
            ki=self.declare_parameter("t_i", 0.0).value,
            kd=self.declare_parameter("t_d", 0.0).value,
            output_limit=self.declare_parameter("t_error_limit", 3.14).value
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

        self.create_subscription(
            Imu,
            "/imu",
            self.imu_callback,
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

    @staticmethod
    def normalize_angle(alpha):
        while alpha > pi:
            alpha -= 2*pi
        while alpha < -pi:
            alpha += 2*pi
        return alpha

    def error_callback(self, msg):
        self.road_error = msg.data

    def state_callback(self, msg):
        if msg.data in [TURN_LEFT, TURN_RIGHT]:
            self.cur_state = msg.data
        else:
            self.active_state = msg.data

    def imu_callback(self, msg):
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quat)
        self.current_yaw = yaw

    def control_loop(self):
        # if self.active_state == STOP and (self.cur_state == STOP):
        #     return
        # elif self.active_state == STOP and (self.cur_state != STOP):
        #     self.active_state = self.cur_state

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        cmd = Twist()

        # -------- FSM --------
        if self.active_state == STOP:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
        elif self.active_state in [STRAIGHT, INTERSECTION]:
            cmd.linear.x = 0.2 if self.active_state == STRAIGHT else 0.15
            if self.cur_state in [TURN_RIGHT, TURN_LEFT]:
                self.center_pid.reset()
                self.active_state = self.cur_state
                self.target_yaw = 150.0 * (pi / 180.0)
                self.target_yaw = self.target_yaw if self.active_state == TURN_RIGHT else -self.target_yaw
                self.intersection_side = self.active_state
            else:
                angular = self.center_pid.compute(self.road_error, dt)
                cmd.angular.z = -angular
                self.cmd_pub.publish(cmd)
        elif self.active_state in [TURN_RIGHT, TURN_LEFT]:
            yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
            if abs(yaw_error) <= self.yaw_threshold:
                fsm_state = Int32()
                fsm_state.data = 4
                self.fsm_state_pub.publish(fsm_state)
                self.cur_state = self.active_state = INTERSECTION
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_pid.compute(yaw_error, dt)
            self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
