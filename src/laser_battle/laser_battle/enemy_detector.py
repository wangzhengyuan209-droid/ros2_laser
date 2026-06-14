#!/usr/bin/env python3
"""Enemy detector: ArUco teammate exclusion, depth + HSV enemy detection.

Pipeline:
  1. ArUco ID=1 → teammate exclusion regions
  2. Depth filter → keep only the closest object in ROI
  3. HSV filter → color matching on depth-filtered area
  4. Shape filter → rectangular objects only
  5. Largest = enemy target
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
        self.declare_parameter('max_area', 300000)
        self.declare_parameter('rect_ratio_threshold', 0.60)
        self.declare_parameter('teammate_margin', 80)
        self.declare_parameter('teammate_marker_ids', [0])
        self.declare_parameter('depth_range', 500)     # mm, range beyond closest point

        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.rect_ratio = self.get_parameter('rect_ratio_threshold').value
        self.margin = self.get_parameter('teammate_margin').value
        self.marker_ids = set(self.get_parameter('teammate_marker_ids').value)
        self.depth_range = self.get_parameter('depth_range').value

        self.bridge = CvBridge()

        # ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict)

        # Enemy color HSV range
        self.color_lower = np.array([70, 30, 40])
        self.color_upper = np.array([125, 240, 220])

        # State
        self.latest_depth = None
        self.last_enemy_pos = None    # (cx, cy) last known position
        self.hold_counter = 0         # frames to keep showing last position
        self.hold_frames = 5          # hold for 5 frames after lost

        # Subscribers
        self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.image_callback, 10)
        self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw',
            self.depth_callback, 10)

        # Publishers
        self.enemy_pub = self.create_publisher(Point, '/enemy_position', 10)
        self.detected_pub = self.create_publisher(Bool, '/enemy_detected', 10)
        self.debug_pub = self.create_publisher(Image, '/enemy_debug_image', 10)
        self.mask_pub = self.create_publisher(Image, '/enemy_mask_image', 10)

        self.get_logger().info('Enemy detector started (depth + HSV)')

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception:
            pass

    def _get_teammate_regions(self, cv_image, hsv, corners, ids, h, w):
        raw_regions = []
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in self.marker_ids:
                    pts = corners[i][0]
                    x_min = int(np.min(pts[:, 0]))
                    y_min = int(np.min(pts[:, 1]))
                    x_max = int(np.max(pts[:, 0]))
                    y_max = int(np.max(pts[:, 1]))
                    raw_regions.append((x_min, y_min, x_max, y_max))

        expanded = []
        for x_min, y_min, x_max, y_max in raw_regions:
            expanded.append((
                max(0, x_min - self.margin),
                max(0, y_min - self.margin),
                min(w, x_max + self.margin),
                min(h, y_max + self.margin),
            ))
        return expanded

    def _is_in_teammate_region(self, bx, by, bw, bh, regions):
        for x_min, y_min, x_max, y_max in regions:
            if bx < x_max and bx + bw > x_min and by < y_max and by + bh > y_min:
                return True
        return False

    def _group_nearby(self, candidates, dist_thresh=80):
        used = [False] * len(candidates)
        groups = []
        for i, c in enumerate(candidates):
            if used[i]:
                continue
            group = [c]
            used[i] = True
            ci_x, ci_y = c[2], c[3]
            for j in range(i + 1, len(candidates)):
                if used[j]:
                    continue
                cj_x, cj_y = candidates[j][2], candidates[j][3]
                if abs(ci_x - cj_x) < dist_thresh and abs(ci_y - cj_y) < dist_thresh:
                    group.append(candidates[j])
                    used[j] = True
            groups.append(group)
        return groups

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image convert failed: {e}')
            return

        h, w = cv_image.shape[:2]
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Step 1: ArUco → teammate exclusion regions
        corners, ids, _ = self.aruco_detector.detectMarkers(cv_image)
        teammate_regions = self._get_teammate_regions(cv_image, hsv, corners, ids, h, w)

        # Step 2: ROI (center ± 1/6, total 1/3 height)
        roi_top = h // 2 - h // 6
        roi_bottom = h // 2 + h // 6

        # Step 3: HSV thresholding
        hsv_mask = cv2.inRange(hsv, self.color_lower, self.color_upper)

        # Step 4: Depth filter — keep only the closest object
        # ROI-only HSV mask (default)
        roi_hsv_mask = np.zeros_like(hsv_mask)
        roi_hsv_mask[roi_top:roi_bottom, :] = hsv_mask[roi_top:roi_bottom, :]

        # If depth available, use HSV-matching region's depth as reference
        combined_mask = roi_hsv_mask
        if self.latest_depth is not None and self.latest_depth.shape[:2] == (h, w):
            hsv_pixels_depth = self.latest_depth[roi_top:roi_bottom, :][
                roi_hsv_mask[roi_top:roi_bottom, :] > 0]
            hsv_valid_depths = hsv_pixels_depth[
                (hsv_pixels_depth > 0) & (hsv_pixels_depth < 10000)]
            self.get_logger().info(
                f'DEBUG: hsv_white={np.count_nonzero(roi_hsv_mask)}, '
                f'hsv_valid_depth={hsv_valid_depths.size}',
                throttle_duration_sec=2)
            if hsv_valid_depths.size > 0:
                ref_depth = int(np.median(hsv_valid_depths))
                min_depth = max(0, ref_depth - self.depth_range // 2)
                max_depth = ref_depth + self.depth_range // 2
                self.get_logger().info(
                    f'DEBUG: ref_depth={ref_depth}, range=[{min_depth},{max_depth}]',
                    throttle_duration_sec=2)
                depth_mask = np.zeros_like(hsv_mask)
                depth_mask[roi_top:roi_bottom, :] = (
                    (self.latest_depth[roi_top:roi_bottom, :] >= min_depth) &
                    (self.latest_depth[roi_top:roi_bottom, :] <= max_depth)
                ).astype(np.uint8) * 255
                combined_mask = cv2.bitwise_and(roi_hsv_mask, depth_mask)

        # Morphological: open (remove noise) + close (fill holes)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

        # Publish binary mask
        mask_rgb = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask_rgb, 'bgr8'))

        # Step 5: Find contours
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area or area > self.max_area:
                continue
            bx, by, bw, bh = cv2.boundingRect(cnt)
            bbox_area = bw * bh
            if bbox_area == 0:
                continue
            if bh >= bw:
                continue
            if bw > bh * 3:
                continue
            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            if self._is_in_teammate_region(bx, by, bw, bh, teammate_regions):
                continue

            roi_v = hsv[by:by+bh, bx:bx+bw, 2]
            mean_v = float(np.mean(roi_v)) if roi_v.size > 0 else 0.0
            candidates.append((cnt, area, cx, cy, bx, by, bw, bh, mean_v))

        # Step 6: Group nearby, pick brightest
        if candidates:
            groups = self._group_nearby(candidates, dist_thresh=80)
            group_reps = []
            for group in groups:
                brightest = max(group, key=lambda x: x[8])
                group_reps.append(brightest)
            candidates = group_reps

        # Step 7: Hold mechanism — keep last known position for a few frames
        det = Bool()
        debug_img = cv_image.copy()
        center_x = w // 2

        cv2.line(debug_img, (center_x, 0), (center_x, h), (255, 255, 0), 1)
        cv2.line(debug_img, (0, roi_top), (w, roi_top), (255, 255, 0), 1)
        cv2.line(debug_img, (0, roi_bottom), (w, roi_bottom), (255, 255, 0), 1)

        if candidates:
            best = max(candidates, key=lambda x: x[1])
            cnt, area, cx, cy, bx, by, bw, bh, _ = best
            self.last_enemy_pos = (cx, cy, bx, by, bw, bh)
            self.hold_counter = self.hold_frames
        elif self.last_enemy_pos is not None and self.hold_counter > 0:
            cx, cy, bx, by, bw, bh = self.last_enemy_pos
            self.hold_counter -= 1
        else:
            self.last_enemy_pos = None
            cx = None

        if cx is not None:
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
            cv2.putText(debug_img, 'NO TARGET',
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
