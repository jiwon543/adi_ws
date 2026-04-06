#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Obstacle Perception Node
- LiDAR 전방 ±scan_angle 스캔
- 섹터별 최소 거리 계산
- best gap 방향 publish
- 시각화 publish
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Float32, Bool, Int32
from cv_bridge import CvBridge

# ── Global State ──
bridge = CvBridge()

scan_angle = 30.0
safe_distance = 0.5
stop_distance = 0.2
num_sectors = 12

sector_distances = []
sector_angles = []

ranges = None
angle_increment = 0

pub_gap_angle = None
pub_gap_width = None
pub_min_front_dist = None
pub_has_obstacle = None
pub_debug_image = None

vis_size = 400
last_gap_angle = 0.0
last_gap_width = 0


def init_sectors():
    global sector_distances, sector_angles
    total_angle = scan_angle * 2
    sector_size = total_angle / num_sectors
    sector_angles = []
    for i in range(num_sectors):
        angle = scan_angle - sector_size * (i + 0.5)
        sector_angles.append(angle)
    sector_distances = [10.0] * num_sectors


def calculate_sector_distances():
    global sector_distances
    if ranges is None or angle_increment == 0:
        return

    total_points = len(ranges)
    center_idx = total_points // 2
    points_per_degree = 1.0 / np.degrees(angle_increment)
    sector_size_deg = (scan_angle * 2) / num_sectors

    for i in range(num_sectors):
        start_angle = -scan_angle + sector_size_deg * i
        end_angle = start_angle + sector_size_deg

        start_idx = int(center_idx + start_angle * points_per_degree)
        end_idx = int(center_idx + end_angle * points_per_degree)

        start_idx = max(0, min(start_idx, total_points - 1))
        end_idx = max(0, min(end_idx, total_points - 1))
        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx

        sector_ranges = ranges[start_idx:end_idx + 1]
        valid = sector_ranges[(sector_ranges > 0.01) & (sector_ranges < 10.0)]
        sector_distances[i] = float(np.min(valid)) if len(valid) > 0 else 10.0


def find_best_gap():
    is_open = [d >= safe_distance for d in sector_distances]

    gaps = []
    gap_start = None
    for i, o in enumerate(is_open):
        if o and gap_start is None:
            gap_start = i
        elif not o and gap_start is not None:
            gaps.append((gap_start, i - 1))
            gap_start = None
    if gap_start is not None:
        gaps.append((gap_start, len(is_open) - 1))

    if not gaps:
        max_idx = int(np.argmax(sector_distances))
        return sector_angles[max_idx], 1

    best = max(gaps, key=lambda g: g[1] - g[0])
    gs, ge = best
    width = ge - gs + 1

    center = (gs + ge) / 2.0
    if center == int(center):
        angle = sector_angles[int(center)]
    else:
        lo = int(center)
        hi = min(lo + 1, len(sector_angles) - 1)
        r = center - lo
        angle = sector_angles[lo] * (1 - r) + sector_angles[hi] * r

    return angle, width


def get_min_front_distance():
    center_sectors = num_sectors // 3
    start = (num_sectors - center_sectors) // 2
    end = start + center_sectors
    front = sector_distances[start:end]
    return min(front) if front else 10.0


def has_obstacle_in_front():
    return get_min_front_distance() < safe_distance


def publish_debug_image():
    global last_gap_angle, last_gap_width
    size = vis_size
    img = np.zeros((size, size, 3), dtype=np.uint8)
    cx, cy = size // 2, size - 50

    for r_m in [0.5, 1.0, 1.5, 2.0]:
        cv2.circle(img, (cx, cy), int(r_m * 100), (40, 40, 40), 1)

    for i, (ang, dist) in enumerate(zip(sector_angles, sector_distances)):
        rad = np.radians(90 - ang)
        d = min(dist, 2.0)
        r = int(d * 100)
        ex = int(cx + r * np.cos(rad))
        ey = int(cy - r * np.sin(rad))

        if dist >= safe_distance:
            color = (0, 255, 0)
        elif dist >= stop_distance:
            color = (0, 165, 255)
        else:
            color = (0, 0, 255)

        cv2.line(img, (cx, cy), (ex, ey), color, 2)
        if dist < 2.0:
            cv2.circle(img, (ex, ey), 4, color, -1)

    # best gap arrow
    gap_rad = np.radians(90 - last_gap_angle)
    gx = int(cx + 150 * np.cos(gap_rad))
    gy = int(cy - 150 * np.sin(gap_rad))
    cv2.arrowedLine(img, (cx, cy), (gx, gy), (255, 255, 0), 3, tipLength=0.2)

    cv2.circle(img, (cx, cy), 8, (255, 255, 255), -1)

    cv2.rectangle(img, (0, 0), (size, 50), (0, 0, 0), -1)
    cv2.putText(img, "OBSTACLE PERCEPT", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(img, f"Gap: {last_gap_angle:.1f}deg  Front: {get_min_front_distance():.2f}m",
                (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

    try:
        pub_debug_image.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
    except Exception as e:
        rospy.logwarn_throttle(5, f"[obs_percept] vis error: {e}")


def scan_callback(msg):
    global ranges, angle_increment, last_gap_angle, last_gap_width

    ranges = np.array(msg.ranges)
    angle_increment = msg.angle_increment

    calculate_sector_distances()

    gap_angle, gap_width = find_best_gap()
    last_gap_angle = gap_angle
    last_gap_width = gap_width

    min_front = get_min_front_distance()
    obstacle = has_obstacle_in_front()

    pub_gap_angle.publish(Float32(data=gap_angle))
    pub_gap_width.publish(Int32(data=gap_width))
    pub_min_front_dist.publish(Float32(data=min_front))
    pub_has_obstacle.publish(Bool(data=obstacle))


def vis_timer_callback(event):
    if ranges is not None:
        publish_debug_image()


def main():
    global scan_angle, safe_distance, stop_distance, num_sectors
    global pub_gap_angle, pub_gap_width, pub_min_front_dist
    global pub_has_obstacle, pub_debug_image

    rospy.init_node('obstacle_perception_node')

    scan_angle = rospy.get_param('~scan_angle', 30.0)
    safe_distance = rospy.get_param('~safe_distance', 0.5)
    stop_distance = rospy.get_param('~stop_distance', 0.2)
    num_sectors = rospy.get_param('~num_sectors', 12)

    init_sectors()

    pub_gap_angle = rospy.Publisher('/obstacle/gap_angle', Float32, queue_size=1)
    pub_gap_width = rospy.Publisher('/obstacle/gap_width', Int32, queue_size=1)
    pub_min_front_dist = rospy.Publisher('/obstacle/min_front_dist', Float32, queue_size=1)
    pub_has_obstacle = rospy.Publisher('/obstacle/has_obstacle', Bool, queue_size=1)
    pub_debug_image = rospy.Publisher('/limo/obstacle/debug', Image, queue_size=1)

    rospy.Subscriber('/scan', LaserScan, scan_callback, queue_size=1)

    rospy.Timer(rospy.Duration(0.1), vis_timer_callback)

    rospy.loginfo(f"[obs_percept] started  ±{scan_angle}° {num_sectors}sectors")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
