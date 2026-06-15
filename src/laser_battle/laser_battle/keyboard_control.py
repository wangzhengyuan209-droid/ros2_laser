#!/usr/bin/env python3
"""Keyboard teleop node using curses.

Controls:
  W/S     - speed +/- 0.1 m/s
  A/D     - turn left/right (toggle)
  F       - fire laser (toggle)
  R       - auto-aim mode (turning only)
  E       - full auto mode (aim + follow 40-80cm)
  SPACE   - emergency stop
  ESC     - quit
"""

import curses
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')

        self.declare_parameter('speed_step', 0.25)
        self.declare_parameter('angular_speed', 0.9)
        self.declare_parameter('max_linear_speed', 1.5)

        self.speed_step = self.get_parameter('speed_step').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.max_linear = self.get_parameter('max_linear_speed').value

        self.current_speed = 0.0

        self.cmd_vel_pub = self.create_publisher(Twist, '/manual_cmd_vel', 10)
        self.laser_pub = self.create_publisher(Bool, '/laser_fire', 10)
        self.auto_aim_pub = self.create_publisher(Bool, '/auto_aim_enable', 10)
        self.full_auto_pub = self.create_publisher(Bool, '/full_auto_enable', 10)
        self.confirm_pub = self.create_publisher(Bool, '/confirm_target', 10)

        self.auto_aim = False
        self.full_auto = False
        self.laser_on = False
        self.confirmed = False
        self.under_attack = False
        self.attack_hold_time = 0
        self.attack_hold_duration = 0.5
        self.turn_state = 'stop'

        self.create_subscription(Bool, '/auto_aim_enable', self.auto_aim_cb, 10)
        self.create_subscription(Bool, '/under_attack', self.attack_cb, 10)
        self.create_subscription(Bool, '/target_confirmed', self.confirm_status_cb, 10)

        self.get_logger().info('Keyboard control started')

    def auto_aim_cb(self, msg):
        self.auto_aim = msg.data

    def attack_cb(self, msg):
        if msg.data:
            self.attack_hold_time = self.get_clock().now()
            self.under_attack = True

    def confirm_status_cb(self, msg):
        self.confirmed = msg.data
        # Ignore False - let _check_attack handle the timeout

    def _check_attack(self):
        if self.under_attack:
            elapsed = (self.get_clock().now() - self.attack_hold_time).nanoseconds / 1e9
            if elapsed > self.attack_hold_duration:
                self.under_attack = False

    def run(self, stdscr):
        stdscr.nodelay(True)
        stdscr.timeout(100)
        curses.curs_set(0)

        self._print_header(stdscr)

        while rclpy.ok():
            key = stdscr.getch()

            if key == ord('w') or key == curses.KEY_UP:
                self.current_speed = min(self.max_linear, self.current_speed + self.speed_step)
            elif key == ord('s') or key == curses.KEY_DOWN:
                self.current_speed = max(-self.max_linear, self.current_speed - self.speed_step)
            elif key == ord('a') or key == curses.KEY_LEFT:
                self.turn_state = 'stop' if self.turn_state == 'left' else 'left'
            elif key == ord('d') or key == curses.KEY_RIGHT:
                self.turn_state = 'stop' if self.turn_state == 'right' else 'right'
            elif key == ord(' '):
                self.current_speed = 0.0
                self.turn_state = 'stop'
                self.auto_aim = False
                self.full_auto = False
                self._publish_auto_aim(False)
                self._publish_full_auto(False)
            elif key == 27:  # ESC
                self._send_stop()
                self.laser_on = False
                self._publish_laser()
                self._publish_auto_aim(False)
                self._publish_full_auto(False)
                break

            if key == ord('f'):
                self.laser_on = not self.laser_on
                self._publish_laser()

            # R: auto-aim only
            if key == ord('r'):
                if self.full_auto:
                    self.full_auto = False
                    self._publish_full_auto(False)
                self.auto_aim = not self.auto_aim
                self._publish_auto_aim(self.auto_aim)

            # E: full auto (aim + follow)
            if key == ord('e'):
                if self.auto_aim:
                    self.auto_aim = False
                    self._publish_auto_aim(False)
                self.full_auto = not self.full_auto
                self._publish_full_auto(self.full_auto)

            # Q: confirm target (narrow color range) or reset
            if key == ord('q'):
                msg = Bool()
                msg.data = True
                self.confirm_pub.publish(msg)

            twist = Twist()
            twist.linear.x = self.current_speed
            # Reverse turn direction when going backward
            direction = -1.0 if self.current_speed < 0 else 1.0
            if self.turn_state == 'left':
                twist.angular.z = self.angular_speed * direction
            elif self.turn_state == 'right':
                twist.angular.z = -self.angular_speed * direction

            self.cmd_vel_pub.publish(twist)
            self._check_attack()
            self._update_display(stdscr, twist.linear.x, twist.angular.z)
            rclpy.spin_once(self, timeout_sec=0)

        self._send_stop()

    def _publish_laser(self):
        msg = Bool()
        msg.data = self.laser_on
        self.laser_pub.publish(msg)

    def _publish_auto_aim(self, enable):
        msg = Bool()
        msg.data = enable
        self.auto_aim_pub.publish(msg)

    def _publish_full_auto(self, enable):
        msg = Bool()
        msg.data = enable
        self.full_auto_pub.publish(msg)

    def _send_stop(self):
        self.current_speed = 0.0
        self.turn_state = 'stop'
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def _print_header(self, stdscr):
        stdscr.clear()
        stdscr.addstr(0, 0, '=== Laser Battle - Keyboard Control ===', curses.A_BOLD)
        stdscr.addstr(2, 0, '  W/UP    - +0.25 m/s forward')
        stdscr.addstr(3, 0, '  S/DOWN  - -0.25 m/s forward')
        stdscr.addstr(4, 0, '  A/LEFT  - Turn Left (toggle)')
        stdscr.addstr(5, 0, '  D/RIGHT - Turn Right (toggle)')
        stdscr.addstr(6, 0, '  F       - Fire Laser (toggle)')
        stdscr.addstr(7, 0, '  R       - Auto-Aim (turning only)')
        stdscr.addstr(8, 0, '  E       - Full Auto (aim + follow)')
        stdscr.addstr(9, 0, '  SPACE   - Emergency Stop')
        stdscr.addstr(10, 0, '  ESC     - Quit')
        stdscr.refresh()

    def _update_display(self, stdscr, linear, angular):
        try:
            stdscr.addstr(12, 0, f'  Speed:    {self.current_speed:+.2f} m/s    ')
            stdscr.addstr(13, 0, f'  Turn:     {self.turn_state:6s}          ')
            stdscr.addstr(14, 0, f'  Angular:  {angular:+.2f} rad/s  ')

            if self.full_auto:
                mode = 'FULL AUTO'
            elif self.auto_aim:
                mode = 'AUTO-AIM'
            else:
                mode = 'MANUAL'
            stdscr.addstr(16, 0, f'  Mode:     {mode:10s}  ')
            stdscr.addstr(17, 0, f'  Laser:    {"ON" if self.laser_on else "OFF":10s}  ')
            stdscr.addstr(18, 0, f'  Target:   {"LOCKED" if self.confirmed else "BROAD":10s}  ')

            attack_str = 'YES' if self.under_attack else 'NO'
            attr = curses.A_BOLD | curses.A_REVERSE if self.under_attack else 0
            stdscr.addstr(19, 0, f'  Attacked: {attack_str:10s}  ', attr)

            stdscr.refresh()
        except curses.error:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    try:
        curses.wrapper(node.run)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
