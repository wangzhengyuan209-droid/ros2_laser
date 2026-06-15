#!/usr/bin/env python3
"""Auto-aim and full-auto attack controller.

R mode (auto-aim): PID turning only, user controls speed.
E mode (full auto): PID turning + auto follow (40-80cm), user can adjust speed.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool


class AttackController(Node):
    def __init__(self):
        super().__init__('attack_controller')

        self.declare_parameter('img_width', 640)
        self.declare_parameter('dead_zone', 25)
        self.declare_parameter('kp', 0.004)
        self.declare_parameter('ki', 0.0001)
        self.declare_parameter('kd', 0.002)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('target_lost_timeout', 3.0)
        self.declare_parameter('follow_min', 400)     # mm
        self.declare_parameter('follow_max', 800)     # mm
        self.declare_parameter('follow_speed', 0.4)   # m/s
        self.declare_parameter('enable_dodge', True)

        self.img_w = self.get_parameter('img_width').value
        self.dead_zone = self.get_parameter('dead_zone').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.max_angular = self.get_parameter('max_angular').value
        self.lost_timeout = self.get_parameter('target_lost_timeout').value
        self.follow_min = self.get_parameter('follow_min').value
        self.follow_max = self.get_parameter('follow_max').value
        self.follow_speed = self.get_parameter('follow_speed').value
        self.enable_dodge = self.get_parameter('enable_dodge').value

        self.cx = self.img_w // 2

        # State
        self.enemy_x = 0.0
        self.enemy_depth = 0.0  # mm, from pt.z
        self.enemy_detected = False
        self.auto_aim_enabled = False
        self.full_auto_enabled = False
        self.integral = 0.0
        self.prev_error = 0.0
        self.manual_linear = 0.0
        self.manual_angular = 0.0
        self.last_seen_time = self.get_clock().now()

        # Subscribers
        self.create_subscription(Point, '/enemy_position', self.enemy_cb, 10)
        self.create_subscription(Bool, '/enemy_detected', self.detected_cb, 10)
        self.create_subscription(Bool, '/auto_aim_enable', self.auto_aim_cb, 10)
        self.create_subscription(Bool, '/full_auto_enable', self.full_auto_cb, 10)
        self.create_subscription(Twist, '/manual_cmd_vel', self.manual_cb, 10)
        self.create_subscription(Bool, '/under_attack', self.attack_cb, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_pub = self.create_publisher(Bool, '/laser_fire', 10)
        self.auto_aim_pub = self.create_publisher(Bool, '/auto_aim_enable', 10)
        self.full_auto_pub = self.create_publisher(Bool, '/full_auto_enable', 10)

        self.under_attack = False
        self.attack_hold_time = 0.0
        self.attack_hold_duration = 0.8  # hold attack state for 0.5s
        self.dodge_sign = 1
        self.last_dodge_time = self.get_clock().now()
        self.dodge_interval = 0.2

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Attack controller started')

    def detected_cb(self, msg):
        self.enemy_detected = msg.data
        if msg.data:
            self.last_seen_time = self.get_clock().now()

    def enemy_cb(self, msg):
        alpha = 0.3
        self.enemy_x = alpha * msg.x + (1 - alpha) * self.enemy_x
        if msg.z > 0:
            self.enemy_depth = alpha * msg.z + (1 - alpha) * self.enemy_depth

    def auto_aim_cb(self, msg):
        self.auto_aim_enabled = msg.data
        if not msg.data:
            self._reset_pid()

    def full_auto_cb(self, msg):
        self.full_auto_enabled = msg.data
        if not msg.data:
            self._reset_pid()

    def manual_cb(self, msg):
        self.manual_linear = msg.linear.x
        self.manual_angular = msg.angular.z

    def attack_cb(self, msg):
        if msg.data:
            self.attack_hold_time = self.get_clock().now()
            self.under_attack = True

    def _check_attack(self):
        if self.under_attack:
            elapsed = (self.get_clock().now() - self.attack_hold_time).nanoseconds / 1e9
            if elapsed > self.attack_hold_duration:
                self.under_attack = False

    def _reset_pid(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def control_loop(self):
        self._check_attack()

        # Dodge when under attack (works in any mode, can be disabled)
        if self.under_attack and self.enable_dodge:
            twist = Twist()
            twist.linear.x = 0.6   # fast forward
            twist.angular.z = 1.0  # fast turn
            self.cmd_pub.publish(twist)
            self._fire_laser(False)
            return

        active = self.auto_aim_enabled or self.full_auto_enabled

        if not active:
            twist = Twist()
            twist.linear.x = self.manual_linear
            twist.angular.z = self.manual_angular
            self.cmd_pub.publish(twist)
            return

        # Check target lost
        elapsed = (self.get_clock().now() - self.last_seen_time).nanoseconds / 1e9
        if elapsed > self.lost_timeout and not self.enemy_detected:
            self.get_logger().warn(f'Target lost {elapsed:.1f}s, disabling')
            self.auto_aim_enabled = False
            self.full_auto_enabled = False
            self._fire_laser(False)
            self._publish_state()
            return

        twist = Twist()

        # Auto follow distance (E mode only)
        if self.full_auto_enabled and self.enemy_detected and self.enemy_depth > 0:
            if self.enemy_depth < self.follow_min:
                auto_linear = -self.follow_speed  # too close, reverse
            elif self.enemy_depth > self.follow_max:
                auto_linear = self.follow_speed   # too far, advance
            else:
                auto_linear = 0.0                 # in range, stop
            twist.linear.x = auto_linear + self.manual_linear
        else:
            twist.linear.x = self.manual_linear

        # PID auto-aim
        if not self.enemy_detected:
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self._fire_laser(False)
            self._reset_pid()
            return

        error_x = self.enemy_x - self.cx
        self.integral += error_x * 0.05
        derivative = (error_x - self.prev_error) / 0.05
        self.prev_error = error_x
        self.integral = max(-500.0, min(500.0, self.integral))

        angular_z = -(self.kp * error_x + self.ki * self.integral + self.kd * derivative)
        angular_z = max(-self.max_angular, min(self.max_angular, angular_z))

        twist.angular.z = angular_z + self.manual_angular
        twist.angular.z = max(-self.max_angular, min(self.max_angular, twist.angular.z))

        self.cmd_pub.publish(twist)

        if abs(error_x) < self.dead_zone:
            self._fire_laser(True)
        else:
            self._fire_laser(False)

    def _publish_state(self):
        msg = Bool()
        msg.data = False
        self.auto_aim_pub.publish(msg)
        self.full_auto_pub.publish(msg)

    def _fire_laser(self, on):
        msg = Bool()
        msg.data = on
        self.laser_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AttackController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
