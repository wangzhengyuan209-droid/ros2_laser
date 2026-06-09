#!/usr/bin/env python3
"""Keyboard teleop node using curses - Incremental speed mode.

Controls:
  W       - +0.3 m/s forward
  S       - -0.3 m/s forward
  A       - turn left (toggle)
  D       - turn right (toggle)
  SPACE   - emergency stop (all off)
  F       - fire laser (toggle)
  R       - toggle auto-aim mode
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

        self.declare_parameter('speed_step', 0.1)
        self.declare_parameter('angular_speed', 0.7)
        self.declare_parameter('max_linear_speed', 2.0)

        self.speed_step = self.get_parameter('speed_step').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.max_linear = self.get_parameter('max_linear_speed').value

        self.current_speed = 0.0  # accumulated forward speed

        self.cmd_vel_pub = self.create_publisher(Twist, '/manual_cmd_vel', 10)
        self.laser_pub = self.create_publisher(Bool, '/laser_fire', 10)
        self.auto_aim_pub = self.create_publisher(Bool, '/auto_aim_enable', 10)
        self.lock_pub = self.create_publisher(Bool, '/lock_target', 10)

        self.auto_aim = False
        self.laser_on = False

        # Movement state: 'stop', 'forward', 'backward', 'left', 'right'
        self.turn_state = 'stop'  # 'stop', 'left', 'right'

        self.create_subscription(Bool, '/auto_aim_enable', self.auto_aim_feedback_cb, 10)

        self.get_logger().info('Keyboard control started (toggle mode)')

    def auto_aim_feedback_cb(self, msg):
        self.auto_aim = msg.data

    def run(self, stdscr):
        stdscr.nodelay(True)
        stdscr.timeout(100)
        curses.curs_set(0)

        self._print_header(stdscr)

        while rclpy.ok():
            key = stdscr.getch()

            # W/S: incremental speed
            if key == ord('w') or key == curses.KEY_UP:
                self.current_speed = min(self.max_linear, self.current_speed + self.speed_step)
            elif key == ord('s') or key == curses.KEY_DOWN:
                self.current_speed = max(-self.max_linear, self.current_speed - self.speed_step)
            # A/D: toggle turn
            elif key == ord('a') or key == curses.KEY_LEFT:
                self.turn_state = 'stop' if self.turn_state == 'left' else 'left'
            elif key == ord('d') or key == curses.KEY_RIGHT:
                self.turn_state = 'stop' if self.turn_state == 'right' else 'right'
            elif key == ord(' '):
                self.current_speed = 0.0
                self.turn_state = 'stop'
                self.laser_on = False
                self._publish_laser()
            elif key == 27:  # ESC
                self._send_stop()
                self.laser_on = False
                self._publish_laser()
                self._send_auto_aim(False)
                break

            # Laser toggle
            if key == ord('f'):
                self.laser_on = not self.laser_on
                self._publish_laser()

            # Auto-aim toggle
            if key == ord('r'):
                self.auto_aim = not self.auto_aim
                aim_msg = Bool()
                aim_msg.data = self.auto_aim
                self.auto_aim_pub.publish(aim_msg)
                lock_msg = Bool()
                lock_msg.data = self.auto_aim
                self.lock_pub.publish(lock_msg)

            # Build and publish cmd_vel
            twist = Twist()
            twist.linear.x = self.current_speed
            if self.turn_state == 'left':
                twist.angular.z = self.angular_speed
            elif self.turn_state == 'right':
                twist.angular.z = -self.angular_speed

            self.cmd_vel_pub.publish(twist)
            self._update_display(stdscr, twist.linear.x, twist.angular.z)
            rclpy.spin_once(self, timeout_sec=0)

        self._send_stop()

    def _publish_laser(self):
        msg = Bool()
        msg.data = self.laser_on
        self.laser_pub.publish(msg)

    def _send_stop(self):
        self.current_speed = 0.0
        self.turn_state = 'stop'
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def _send_auto_aim(self, enable):
        msg = Bool()
        msg.data = enable
        self.auto_aim_pub.publish(msg)

    def _print_header(self, stdscr):
        stdscr.clear()
        stdscr.addstr(0, 0, '=== Laser Battle - Keyboard Control ===', curses.A_BOLD)
        stdscr.addstr(2, 0, '  W/UP    - +0.3 m/s forward')
        stdscr.addstr(3, 0, '  S/DOWN  - -0.3 m/s forward')
        stdscr.addstr(4, 0, '  A/LEFT  - Turn Left (toggle)')
        stdscr.addstr(5, 0, '  D/RIGHT - Turn Right (toggle)')
        stdscr.addstr(6, 0, '  SPACE   - Emergency Stop')
        stdscr.addstr(7, 0, '  F       - Fire Laser (toggle)')
        stdscr.addstr(8, 0, '  R       - Toggle Auto-Aim')
        stdscr.addstr(9, 0, '  ESC     - Quit')
        stdscr.refresh()

    def _update_display(self, stdscr, linear, angular):
        try:
            stdscr.addstr(12, 0, f'  Speed:    {self.current_speed:+.2f} m/s    ')
            stdscr.addstr(13, 0, f'  Turn:     {self.turn_state:6s}          ')
            stdscr.addstr(14, 0, f'  Angular:  {angular:+.2f} rad/s  ')
            stdscr.addstr(15, 0, f'  Step:     {self.speed_step:.2f} m/s     ')
            stdscr.addstr(16, 0, f'  Max:      {self.max_linear:.2f} m/s     ')

            status = 'MOVING' if abs(self.current_speed) > 0.01 or self.turn_state != 'stop' else 'STOPPED'
            stdscr.addstr(18, 0, f'  Status:   {status:10s}  ')
            stdscr.addstr(19, 0, f'  Laser:    {"ON" if self.laser_on else "OFF":10s}  ')
            stdscr.addstr(20, 0, f'  Auto-Aim: {"ON" if self.auto_aim else "OFF":10s}  ')
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
