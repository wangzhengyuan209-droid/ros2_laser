#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import LaserScan
import math


class FollowerController(Node):
    """State machine: IDLE → FOLLOWING → APPROACH_HAND → MEDICINE_READY."""

    IDLE = 0
    FOLLOWING = 1
    APPROACH_HAND = 2
    MEDICINE_READY = 3
    OBSTACLE_AVOID = 4
    STATE_NAMES = ['IDLE', 'FOLLOWING', 'APPROACH_HAND', 'MEDICINE_READY', 'OBSTACLE_AVOID']

    def __init__(self):
        super().__init__('follower_controller')

        # ========== Parameters ==========
        self.declare_parameter('follow_distance', 1.5)
        self.declare_parameter('approach_distance', 0.3)
        self.declare_parameter('kp_dist', 0.5)
        self.declare_parameter('kp_angle', 1.0)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 0.8)
        self.declare_parameter('obstacle_threshold', 0.3)
        self.declare_parameter('lost_timeout', 3.0)
        self.declare_parameter('control_hz', 20.0)

        self.follow_dist = self.get_parameter('follow_distance').value
        self.approach_dist = self.get_parameter('approach_distance').value
        self.kp_dist = self.get_parameter('kp_dist').value
        self.kp_angle = self.get_parameter('kp_angle').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.obs_threshold = self.get_parameter('obstacle_threshold').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.control_hz = self.get_parameter('control_hz').value

        # ========== State ==========
        self.state = self.IDLE
        self.person_distance = 0.0
        self.person_angle = 0.0    # radians, + = left, - = right
        self.hand_distance = 0.0
        self.hand_angle = 0.0
        self.is_fallen = False
        self.person_visible = False
        self.hand_visible = False
        self.obstacle_detected = False

        # ========== Subscribers ==========
        self.create_subscription(PointStamped, '/person_position', self.person_callback, 10)
        self.create_subscription(Float32, '/person_distance', self.distance_callback, 10)
        self.create_subscription(Bool, '/fall_detected', self.fall_callback, 10)
        self.create_subscription(PointStamped, '/person_hand_position', self.hand_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # ========== Publisher ==========
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ========== Control Timer ==========
        dt = 1.0 / self.control_hz
        self.create_timer(dt, self.control_loop)

        self.get_logger().info('Follower Controller started')

    # ---- Callbacks (no TF needed, use raw camera frame) ----

    def person_callback(self, msg: PointStamped):
        self.person_visible = True
        # msg is in camera_link: z = forward, x = lateral, y = up/down
        d = math.sqrt(msg.point.x**2 + msg.point.z**2)
        if d > 0.01:
            self.person_angle = math.atan2(msg.point.x, msg.point.z)

    def distance_callback(self, msg: Float32):
        self.person_distance = msg.data

    def fall_callback(self, msg: Bool):
        self.is_fallen = msg.data
        if self.is_fallen and self.state == self.FOLLOWING:
            self.set_state(self.APPROACH_HAND)
            self.get_logger().info('Fall detected! Approaching hand...')

    def hand_callback(self, msg: PointStamped):
        self.hand_visible = True
        d = math.sqrt(msg.point.x**2 + msg.point.z**2)
        if d > 0.01:
            self.hand_distance = d
            self.hand_angle = math.atan2(msg.point.x, msg.point.z)

    def scan_callback(self, msg: LaserScan):
        center = len(msg.ranges) // 2
        half_range = int(len(msg.ranges) * 30 / 360)
        front = msg.ranges[center - half_range : center + half_range]
        valid = [r for r in front if msg.range_min < r < msg.range_max]
        self.obstacle_detected = any(r < self.obs_threshold for r in valid) if valid else False

    # ---- State Machine ----

    def set_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f'State: {self.STATE_NAMES[self.state]} → {self.STATE_NAMES[new_state]}')
            self.state = new_state

    def control_loop(self):
        twist = Twist()

        if self.state == self.IDLE:
            if self.person_visible:
                self.set_state(self.FOLLOWING)

        elif self.state == self.OBSTACLE_AVOID:
            if not self.obstacle_detected:
                if self.is_fallen:
                    self.set_state(self.APPROACH_HAND)
                else:
                    self.set_state(self.FOLLOWING)

        elif self.state == self.FOLLOWING:
            if not self.person_visible:
                pass  # twist = 0
            elif self.obstacle_detected:
                self.set_state(self.OBSTACLE_AVOID)
            else:
                # Distance error
                error = self.person_distance - self.follow_dist
                twist.linear.x = max(-self.max_linear, min(self.max_linear, self.kp_dist * error))

                # Don't reverse when already close
                if self.person_distance < self.follow_dist * 0.3:
                    twist.linear.x = 0.0

                # Angular: turn toward person
                twist.angular.z = max(-self.max_angular, min(self.max_angular, -self.kp_angle * self.person_angle))

        elif self.state == self.APPROACH_HAND:
            if not self.hand_visible and self.person_visible:
                # Fallback: approach body center
                error = self.person_distance - self.approach_dist
                twist.linear.x = max(-self.max_linear, min(self.max_linear, self.kp_dist * error))
                twist.angular.z = max(-self.max_angular, min(self.max_angular, -self.kp_angle * self.person_angle))
            elif self.obstacle_detected:
                self.set_state(self.OBSTACLE_AVOID)
            else:
                error = self.hand_distance - self.approach_dist
                twist.linear.x = max(-self.max_linear, min(self.max_linear, self.kp_dist * error))
                twist.angular.z = max(-self.max_angular, min(self.max_angular, -self.kp_angle * self.hand_angle))

                if self.hand_distance <= self.approach_dist * 1.2:
                    self.set_state(self.MEDICINE_READY)

        elif self.state == self.MEDICINE_READY:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if self.person_visible and not self.is_fallen:
                self.set_state(self.IDLE)

        self.cmd_pub.publish(twist)

    def destroy_node(self):
        self.cmd_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FollowerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
