#!/usr/bin/env python3
"""Enemy detector: ArUco + red light teammate exclusion, blue/green enemy detection.

Teammate = blue car with ArUco ID=1 marker + red LED light.
If a blue/green rectangle is near an ArUco marker OR red light → teammate (no box drawn).
Remaining blue/green rectangles → enemy (red box).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class EnemyDetector(Node):
    def __init__(self):
        super().__init__('enemy_detector')

        self.declare_parameter('min_area', 300)
        self.declare_parameter('max_area', 250000)
        self.declare_parameter('rect_ratio_threshold', 0.60)
        self.declare_parameter('teammate_margin', 80)
        self.declare_parameter('teammate_marker_ids', [1])
        self.declare_parameter('confirm_frames', 3)

        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.rect_ratio = self.get_parameter('rect_ratio_threshold').value
        self.margin = self.get_parameter('teammate_margin').value
        self.marker_ids = set(self.get_parameter('teammate_marker_ids').value)
        self.confirm_frames = self.get_parameter('confirm_frames').value

        self.bridge = CvBridge()

        # ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict)

        # Blue/green HSV range (enemy)
        self.color_lower = np.array([35, 50, 50])
        self.color_upper = np.array([130, 255, 255])

        # Red LED HSV range (narrow, bright LED only)
        self.red_lower1 = np.array([0, 150, 150])
        self.red_upper1 = np.array([8, 255, 255])
        self.red_lower2 = np.array([172, 150, 150])
        self.red_upper2 = np.array([180, 255, 255])

        # Temporal filtering
        self.detection_count = 0

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.image_callback, 10)

        # Publishers
        self.enemy_pub = self.create_publisher(Point, '/enemy_position', 10)
        self.detected_pub = self.create_publisher(Bool, '/enemy_detected', 10)
        self.debug_pub = self.create_publisher(Image, '/enemy_debug_image', 10)

        self.get_logger().info('Enemy detector started (ArUco ID=1 + red LED teammate)')

    def _get_teammate_regions(self, cv_image, hsv, corners, ids, h, w):
        """Get teammate exclusion regions from ArUco markers and red LED lights."""
        raw_regions = []

        # ArUco markers
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in self.marker_ids:
                    pts = corners[i][0]
                    x_min = int(np.min(pts[:, 0]))
                    y_min = int(np.min(pts[:, 1]))
                    x_max = int(np.max(pts[:, 0]))
                    y_max = int(np.max(pts[:, 1]))
                    raw_regions.append((x_min, y_min, x_max, y_max))

        # Red LED lights
        red_mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in red_contours:
            area = cv2.contourArea(cnt)
            if area < 30:
                continue
            bx, by, bw, bh = cv2.boundingRect(cnt)
            raw_regions.append((bx, by, bx + bw, by + bh))

        # Merge overlapping regions, then expand by margin
        merged = self._merge_regions(raw_regions)
        expanded = []
        for x_min, y_min, x_max, y_max in merged:
            expanded.append((
                max(0, x_min - self.margin),
                max(0, y_min - self.margin),
                min(w, x_max + self.margin),
                min(h, y_max + self.margin),
            ))
        return expanded

    def _merge_regions(self, regions):
        """Merge overlapping or nearby regions into larger ones."""
        if not regions:
            return []
        # Sort by x_min
        regions = sorted(regions, key=lambda r: r[0])
        merged = [regions[0]]
        for x_min, y_min, x_max, y_max in regions[1:]:
            px_min, py_min, px_max, py_max = merged[-1]
            # Check overlap or close proximity (within 50px)
            if x_min <= px_max + 50 and y_min <= py_max + 50 and y_max >= py_min - 50:
                merged[-1] = (
                    min(px_min, x_min),
                    min(py_min, y_min),
                    max(px_max, x_max),
                    max(py_max, y_max),
                )
            else:
                merged.append((x_min, y_min, x_max, y_max))
        return merged

    def _is_in_teammate_region(self, bx, by, bw, bh, regions):
        """Check if a bounding box overlaps with any teammate exclusion region."""
        for x_min, y_min, x_max, y_max in regions:
            if bx < x_max and bx + bw > x_min and by < y_max and by + bh > y_min:
                return True
        return False

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image convert failed: {e}')
            return

        h, w = cv_image.shape[:2]
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Step 1: ArUco + red light → teammate exclusion regions
        corners, ids, _ = self.aruco_detector.detectMarkers(cv_image)
        teammate_regions = self._get_teammate_regions(cv_image, hsv, corners, ids, h, w)

        # Step 2: Blue/green rectangular detection
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area or area > self.max_area:
                continue
            bx, by, bw, bh = cv2.boundingRect(cnt)
            bbox_area = bw * bh
            if bbox_area == 0:
                continue
            if area / bbox_area < self.rect_ratio:
                continue
            if bh >= bw:
                continue
            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # Step 3: Skip if overlaps with teammate region
            if self._is_in_teammate_region(bx, by, bw, bh, teammate_regions):
                continue

            candidates.append((cnt, area, cx, cy, bx, by, bw, bh))

        # Step 4: Temporal filtering
        if candidates:
            self.detection_count += 1
        else:
            self.detection_count = 0
        confirmed = self.detection_count >= self.confirm_frames

        # Step 5: Debug image
        det = Bool()
        debug_img = cv_image.copy()
        center_x = w // 2

        # Draw frame center
        cv2.line(debug_img, (center_x, 0), (center_x, h), (255, 255, 0), 1)

        if candidates and confirmed:
            best = max(candidates, key=lambda x: x[1])
            cnt, area, cx, cy, bx, by, bw, bh = best
            error_x = cx - center_x

            det.data = True
            pt = Point()
            pt.x = float(cx)
            pt.y = float(cy)
            pt.z = 0.0
            self.enemy_pub.publish(pt)

            cv2.rectangle(debug_img, (bx, by), (bx + bw, by + bh), (0, 0, 255), 2)
            cv2.circle(debug_img, (cx, cy), 6, (0, 0, 255), -1)
            cv2.line(debug_img, (cx, cy), (center_x, cy), (0, 0, 255), 2)
            cv2.putText(debug_img, f'ENEMY ({cx},{cy})',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(debug_img, f'ERR: {error_x:+d}px',
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            det.data = False
            status = 'CONFIRMING...' if candidates else 'NO TARGET'
            cv2.putText(debug_img, status,
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 128, 128), 2)

        self.detected_pub.publish(det)
        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, 'bgr8')
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EnemyDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
