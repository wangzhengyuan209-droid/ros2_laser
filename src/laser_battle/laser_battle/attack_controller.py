#!/usr/bin/env python3
"""Auto-aim attack controller with PID.

When auto-aim is enabled (R key):
- Reads user's manual input for forward/backward
- PID controls turning to keep target centered
- Fires laser when target is aligned
- Auto-disables if target lost for too long
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool


class AttackController(Node):
    def __init__(self):
        super().__init__('attack_controller')

        # Parameters
        self.declare_parameter('img_width', 640)
        self.declare_parameter('img_height', 480)
        self.declare_parameter('dead_zone', 25)
        self.declare_parameter('forward_speed', 0.25)
        self.declare_parameter('kp', 0.004)
        self.declare_parameter('ki', 0.0001)
        self.declare_parameter('kd', 0.002)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('target_lost_timeout', 3.0)  # seconds

        self.img_w = self.get_parameter('img_width').value
        self.img_h = self.get_parameter('img_height').value
        self.dead_zone = self.get_parameter('dead_zone').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.max_angular = self.get_parameter('max_angular').value
        self.lost_timeout = self.get_parameter('target_lost_timeout').value

        self.cx = self.img_w // 2

        # State
        self.enemy_x = 0.0
        self.enemy_detected = False
        self.auto_aim_enabled = False
        self.integral = 0.0
        self.prev_error = 0.0
        self.manual_linear = 0.0    # user's W/S input
        self.manual_angular = 0.0   # user's A/D input
        self.last_seen_time = self.get_clock().now()

        # Dodge state
        self.under_attack = False
        self.dodge_sign = 1          # +1 or -1
        self.last_dodge_time = self.get_clock().now()
        self.dodge_interval = 0.2    # seconds between direction switch

        # Subscribers
        self.create_subscription(Point, '/enemy_position', self.enemy_cb, 10)
        self.create_subscription(Bool, '/enemy_detected', self.detected_cb, 10)
        self.create_subscription(Bool, '/auto_aim_enable', self.auto_aim_cb, 10)
        self.create_subscription(Twist, '/manual_cmd_vel', self.manual_cb, 10)
        self.create_subscription(Bool, '/under_attack', self.attack_cb, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_pub = self.create_publisher(Bool, '/laser_fire', 10)
        self.auto_aim_pub = self.create_publisher(Bool, '/auto_aim_enable', 10)

        # Control loop at 20Hz
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Attack controller started. Press R to enable auto-aim.')

    def enemy_cb(self, msg):
        # EMA low-pass filter to smooth noisy detections
        alpha = 0.3
        self.enemy_x = alpha * msg.x + (1 - alpha) * self.enemy_x

    def detected_cb(self, msg):
        self.enemy_detected = msg.data
        if msg.data:
            self.last_seen_time = self.get_clock().now()

    def auto_aim_cb(self, msg):
        self.auto_aim_enabled = msg.data
        if not msg.data:
            self.integral = 0.0
            self.prev_error = 0.0
            self._fire_laser(False)
            self.get_logger().info('Auto-aim disabled')

    def manual_cb(self, msg):
        self.manual_linear = msg.linear.x
        self.manual_angular = msg.angular.z

    def attack_cb(self, msg):
        self.under_attack = msg.data

    def control_loop(self):
        if not self.auto_aim_enabled:
            # Auto-aim off: pass through manual control directly
            twist = Twist()
            twist.linear.x = self.manual_linear
            twist.angular.z = self.manual_angular
            self.cmd_pub.publish(twist)
            return

        # Check target lost timeout
        elapsed = (self.get_clock().now() - self.last_seen_time).nanoseconds / 1e9
        if elapsed > self.lost_timeout and not self.enemy_detected:
            self.get_logger().warn(f'Target lost for {elapsed:.1f}s, disabling auto-aim')
            self.auto_aim_enabled = False
            self._fire_laser(False)
            # Notify keyboard_control to update its state
            msg = Bool()
            msg.data = False
            self.auto_aim_pub.publish(msg)
            return

        twist = Twist()

        # Forward speed: only move if user explicitly presses W/S
        twist.linear.x = self.manual_linear

        # Dodge: when under attack, rapidly alternate turning direction
        if self.under_attack:
            now = self.get_clock().now()
            elapsed = (now - self.last_dodge_time).nanoseconds / 1e9
            if elapsed > self.dodge_interval:
                self.dodge_sign *= -1
                self.last_dodge_time = now
            twist.angular.z = self.max_angular * self.dodge_sign
            self.cmd_pub.publish(twist)
            self._fire_laser(False)
            return

        if not self.enemy_detected:
            # No target visible: go straight, no turn
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self._fire_laser(False)
            self.integral = 0.0
            self.prev_error = 0.0
            return

        # PID auto-aim
        error_x = self.enemy_x - self.cx

        self.integral += error_x * 0.05
        derivative = (error_x - self.prev_error) / 0.05
        self.prev_error = error_x

        # Anti-windup
        self.integral = max(-500.0, min(500.0, self.integral))

        angular_z = -(self.kp * error_x + self.ki * self.integral + self.kd * derivative)
        angular_z = max(-self.max_angular, min(self.max_angular, angular_z))

        # Blend: user can add manual turning on top of auto-aim
        twist.angular.z = angular_z + self.manual_angular
        twist.angular.z = max(-self.max_angular, min(self.max_angular, twist.angular.z))

        self.cmd_pub.publish(twist)

        # Fire if aligned
        if abs(error_x) < self.dead_zone:
            self._fire_laser(True)
        else:
            self._fire_laser(False)

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
