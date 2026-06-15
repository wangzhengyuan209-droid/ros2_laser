#!/usr/bin/env python3
"""Enemy detector for 2v2 laser battle.

Color-based team detection:
  - my_color: our team color (not attacked)
  - enemy_color: enemy team color (attack)
  - red: hit enemy (attack, takes priority)

Launch parameter: team_color:=green or team_color:=blue
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
        self.declare_parameter('team_color', 'green')
        self.declare_parameter('depth_range', 500)
        self.declare_parameter('include_red', True)

        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.team_color = self.get_parameter('team_color').value
        self.depth_range = self.get_parameter('depth_range').value
        self.include_red = self.get_parameter('include_red').value

        self.bridge = CvBridge()

        # Color ranges
        # Green: H=35-85
        self.green_lower = np.array([35, 40, 40])
        self.green_upper = np.array([85, 255, 255])

        # Blue: H=85-130
        self.blue_lower = np.array([85, 40, 40])
        self.blue_upper = np.array([130, 255, 255])

        # Red: H=0-10 and 170-180 (hit indicator)
        self.red_lower1 = np.array([0, 80, 80])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([170, 80, 80])
        self.red_upper2 = np.array([180, 255, 255])

        # Set attack color based on team
        if self.team_color == 'green':
            self.attack_lower = self.blue_lower
            self.attack_upper = self.blue_upper
            self.get_logger().info('Team: GREEN | Attack: BLUE + RED')
        else:
            self.attack_lower = self.green_lower
            self.attack_upper = self.green_upper
            self.get_logger().info('Team: BLUE | Attack: GREEN + RED')

        # State
        self.latest_depth = None
        self.last_enemy_pos = None
        self.hold_counter = 0
        self.hold_frames = 5

        # State for target confirmation
        self.confirmed = False
        self.current_bbox = None  # (bx, by, bw, bh) of current target
        self.latest_hsv = None

        # Subscribers
        self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.image_callback, 10)
        self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw',
            self.depth_callback, 10)
        self.create_subscription(
            Bool, '/confirm_target', self.confirm_cb, 10)

        # Publishers
        self.enemy_pub = self.create_publisher(Point, '/enemy_position', 10)
        self.detected_pub = self.create_publisher(Bool, '/enemy_detected', 10)
        self.debug_pub = self.create_publisher(Image, '/enemy_debug_image', 10)
        self.mask_pub = self.create_publisher(Image, '/enemy_mask_image', 10)
        self.confirmed_pub = self.create_publisher(Bool, '/target_confirmed', 10)

        self.get_logger().info(f'Enemy detector started (team={self.team_color})')

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception:
            pass

    def confirm_cb(self, msg):
        if not msg.data:
            return
        if self.confirmed:
            # Reset to broad range
            if self.team_color == 'green':
                self.attack_lower = self.blue_lower
                self.attack_upper = self.blue_upper
            else:
                self.attack_lower = self.green_lower
                self.attack_upper = self.green_upper
            self.confirmed = False
            self.get_logger().info('Target reset to broad range')
        elif self.current_bbox is not None and self.latest_hsv is not None:
            bx, by, bw, bh = self.current_bbox
            roi = self.latest_hsv[by:by+bh, bx:bx+bw]
            if roi.size > 0:
                mean_hsv = np.mean(roi.reshape(-1, 3), axis=0)
                margin = np.array([15, 50, 50])
                self.attack_lower = np.clip(mean_hsv - margin, 0, 255).astype(np.uint8)
                self.attack_upper = np.clip(mean_hsv + margin, 0, 255).astype(np.uint8)
                self.confirmed = True
                self.get_logger().info(
                    f'Target confirmed! HSV=({mean_hsv[0]:.0f},{mean_hsv[1]:.0f},{mean_hsv[2]:.0f})')
        # Publish confirmed state
        msg_out = Bool()
        msg_out.data = self.confirmed
        self.confirmed_pub.publish(msg_out)

    def _detect_color(self, hsv, lower, upper, roi_top, roi_bottom):
        """Detect a color within ROI, return mask."""
        mask = cv2.inRange(hsv, lower, upper)
        roi_mask = np.zeros_like(mask)
        roi_mask[roi_top:roi_bottom, :] = mask[roi_top:roi_bottom, :]
        return roi_mask

    def _apply_depth_filter(self, hsv_mask, h, w, roi_top, roi_bottom):
        """Apply depth filter on HSV mask. Returns filtered mask."""
        if self.latest_depth is None or self.latest_depth.shape[:2] != (h, w):
            return hsv_mask

        hsv_pixels_depth = self.latest_depth[roi_top:roi_bottom, :][
            hsv_mask[roi_top:roi_bottom, :] > 0]
        hsv_valid = hsv_pixels_depth[
            (hsv_pixels_depth > 0) & (hsv_pixels_depth < 10000)]

        if hsv_valid.size == 0:
            return hsv_mask

        ref_depth = int(np.median(hsv_valid))
        min_d = max(0, ref_depth - self.depth_range // 2)
        max_d = ref_depth + self.depth_range // 2

        depth_mask = np.zeros_like(hsv_mask)
        depth_mask[roi_top:roi_bottom, :] = (
            (self.latest_depth[roi_top:roi_bottom, :] >= min_d) &
            (self.latest_depth[roi_top:roi_bottom, :] <= max_d)
        ).astype(np.uint8) * 255

        return cv2.bitwise_and(hsv_mask, depth_mask)

    def _find_candidates(self, mask, hsv):
        """Find rectangular candidates from a binary mask."""
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
            if bh >= bw:
                continue
            if bw > bh * 3:
                continue
            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            roi_v = hsv[by:by+bh, bx:bx+bw, 2]
            mean_v = float(np.mean(roi_v)) if roi_v.size > 0 else 0.0
            candidates.append((cnt, area, cx, cy, bx, by, bw, bh, mean_v))
        return candidates

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

    def _pick_best(self, candidates):
        """Group nearby, pick brightest in each group, then largest across groups."""
        if not candidates:
            return None
        groups = self._group_nearby(candidates)
        reps = [max(g, key=lambda x: x[8]) for g in groups]
        return max(reps, key=lambda x: x[1])

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image convert failed: {e}')
            return

        h, w = cv_image.shape[:2]
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        self.latest_hsv = hsv

        roi_top = h // 2 - h // 6
        roi_bottom = h // 2 + h // 6

        # Detect red (hit enemies) - only if include_red
        if self.include_red:
            red_mask = self._detect_color(hsv, self.red_lower1, self.red_upper1, roi_top, roi_bottom)
            red_mask2 = self._detect_color(hsv, self.red_lower2, self.red_upper2, roi_top, roi_bottom)
            red_mask = cv2.bitwise_or(red_mask, red_mask2)
            red_mask = self._apply_depth_filter(red_mask, h, w, roi_top, roi_bottom)
        else:
            red_mask = np.zeros((h, w), dtype=np.uint8)

        # Detect enemy color
        attack_mask = self._detect_color(hsv, self.attack_lower, self.attack_upper, roi_top, roi_bottom)
        attack_mask = self._apply_depth_filter(attack_mask, h, w, roi_top, roi_bottom)

        # Combine: red OR enemy_color
        combined_mask = cv2.bitwise_or(red_mask, attack_mask)

        # Find candidates
        red_candidates = self._find_candidates(red_mask, hsv) if self.include_red else []
        attack_candidates = self._find_candidates(attack_mask, hsv)

        # Red takes priority (only if include_red)
        best = self._pick_best(red_candidates)
        best_color = 'RED'
        if best is None:
            best = self._pick_best(attack_candidates)
            best_color = 'ENEMY'

        # Hold mechanism
        det = Bool()
        debug_img = cv_image.copy()
        center_x = w // 2

        # Crosshair at center
        cx_img, cy_img = w // 2, h // 2
        cv2.line(debug_img, (cx_img - 20, cy_img), (cx_img + 20, cy_img), (0, 255, 0), 1)
        cv2.line(debug_img, (cx_img, cy_img - 20), (cx_img, cy_img + 20), (0, 255, 0), 1)

        if best is not None:
            cnt, area, cx, cy, bx, by, bw, bh, _ = best
            self.last_enemy_pos = (cx, cy, bx, by, bw, bh, best_color)
            self.current_bbox = (bx, by, bw, bh)
            self.hold_counter = self.hold_frames
        elif self.last_enemy_pos is not None and self.hold_counter > 0:
            cx, cy, bx, by, bw, bh, best_color = self.last_enemy_pos
            self.hold_counter -= 1
        else:
            self.last_enemy_pos = None
            self.current_bbox = None
            cx = None

        if cx is not None:
            error_x = cx - center_x
            det.data = True
            pt = Point()
            pt.x = float(cx)
            pt.y = float(cy)
            # Get depth at target position
            if self.latest_depth is not None and self.latest_depth.shape[:2] == (h, w):
                d = self.latest_depth[cy, cx]
                pt.z = float(d) if 0 < d < 10000 else 0.0
            else:
                pt.z = 0.0
            self.enemy_pub.publish(pt)

            box_color = (0, 0, 255) if best_color == 'RED' else (255, 0, 0)
            cv2.rectangle(debug_img, (bx, by), (bx + bw, by + bh), box_color, 2)
            cv2.circle(debug_img, (cx, cy), 6, (0, 0, 255), -1)
            cv2.line(debug_img, (cx, cy), (center_x, cy), (0, 0, 255), 2)
            cv2.putText(debug_img, f'{best_color} ({cx},{cy})',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
            cv2.putText(debug_img, f'ERR: {error_x:+d}px',
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            det.data = False
            cv2.putText(debug_img, 'NO TARGET',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 128, 128), 2)

        self.detected_pub.publish(det)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, 'bgr8'))
        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(
            cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR), 'bgr8'))


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
