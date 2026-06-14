#!/usr/bin/env python3
"""Monitor attack state from STM32 (state_1 field)."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class AttackMonitor(Node):
    def __init__(self):
        super().__init__('attack_monitor')
        self.create_subscription(Bool, '/under_attack', self.callback, 10)
        self.last_state = None
        self.get_logger().info('Attack Monitor started - waiting for /under_attack topic')

    def callback(self, msg):
        state = 'ATTACKED!' if msg.data else 'safe'
        if state != self.last_state:
            self.get_logger().warn(f'State changed: {state}')
            self.last_state = state


def main(args=None):
    rclpy.init(args=args)
    node = AttackMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
