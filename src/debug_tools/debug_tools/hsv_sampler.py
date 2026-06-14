#!/usr/bin/env python3
"""Print HSV value at the center of the camera image."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class HSVSampler(Node):
    def __init__(self):
        super().__init__('hsv_sampler')
        self.bridge = CvBridge()
        self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.callback, 10)
        self.get_logger().info('HSV Sampler started - point camera at target')

    def callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2
        roi = img[cy-5:cy+5, cx-5:cx+5]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mean = np.mean(hsv.reshape(-1, 3), axis=0)
        self.get_logger().info(f'HSV=({mean[0]:.0f}, {mean[1]:.0f}, {mean[2]:.0f})')


def main(args=None):
    rclpy.init(args=args)
    node = HSVSampler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
