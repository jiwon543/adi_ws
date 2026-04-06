#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lane Perception Node
- ROI 내 흰색 차선 검출 (HLS)
- lane_center_x, left_x, right_x publish
- 시각화 이미지 publish
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32, Float32
from cv_bridge import CvBridge

# ── Global State ──
bridge = CvBridge()
_vis_frame_count = 0
VIS_SKIP = 5  # 5프레임에 1번만 시각화 퍼블리시

LANE_LOW = np.array([0, 180, 0])
LANE_HIGH = np.array([180, 255, 60])

roi_top = 300
roi_bottom = 450
lane_offset = 180
img_width = 640
img_height = 480

# PD control
kp = 0.01
kd = 0.005
max_steering = 0.5
prev_error = 0

pub_center_x = None
pub_left_x = None
pub_right_x = None
pub_steering = None
pub_image = None


def detect_lanes(img):
    roi = img[roi_top:roi_bottom, :]
    hls = cv2.cvtColor(roi, cv2.COLOR_BGR2HLS)

    mask = cv2.inRange(hls, LANE_LOW, LANE_HIGH)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    mid = roi.shape[1] // 2
    left_x, right_x = -1, -1

    M_left = cv2.moments(mask[:, :mid])
    if M_left['m00'] > 0:
        left_x = int(M_left['m10'] / M_left['m00'])

    M_right = cv2.moments(mask[:, mid:])
    if M_right['m00'] > 0:
        right_x = int(M_right['m10'] / M_right['m00']) + mid

    return left_x, right_x, mask


def calc_lane_center(left_x, right_x):
    mid = img_width // 2
    if left_x > 0 and right_x > 0:
        return (left_x + right_x) // 2
    elif left_x > 0:
        return left_x + lane_offset
    elif right_x > 0:
        return right_x - lane_offset
    else:
        return mid


def draw_visualization(img, left_x, right_x, lane_center_x):
    vis = img.copy()
    roi_y = (roi_top + roi_bottom) // 2
    center = img_width // 2
    error = lane_center_x - center

    cv2.rectangle(vis, (0, roi_top), (img_width, roi_bottom), (255, 255, 0), 2)
    if left_x > 0:
        cv2.circle(vis, (left_x, roi_y), 10, (255, 0, 0), -1)
    if right_x > 0:
        cv2.circle(vis, (right_x, roi_y), 10, (0, 0, 255), -1)
    cv2.circle(vis, (lane_center_x, roi_y), 10, (0, 255, 0), -1)
    cv2.line(vis, (center, roi_top), (center, roi_bottom), (0, 255, 255), 2)
    cv2.arrowedLine(vis, (center, roi_y), (lane_center_x, roi_y), (0, 255, 0), 3)
    cv2.putText(vis, f"Offset: {error}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(vis, f"L:{left_x} R:{right_x}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    return vis


def image_callback(msg):
    global img_width, img_height, prev_error, _vis_frame_count
    try:
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            return

        img_height, img_width = img.shape[:2]
        left_x, right_x, _ = detect_lanes(img)
        lane_center_x = calc_lane_center(left_x, right_x)

        # PD control
        error = lane_center_x - (img_width // 2)
        derivative = error - prev_error
        steering = float(np.clip(kp * error + kd * derivative,
                                 -max_steering, max_steering))
        prev_error = error

        pub_center_x.publish(Int32(data=lane_center_x))
        pub_left_x.publish(Int32(data=left_x))
        pub_right_x.publish(Int32(data=right_x))
        pub_steering.publish(Float32(data=steering))

        _vis_frame_count += 1
        if _vis_frame_count % VIS_SKIP == 0:
            vis = draw_visualization(img, left_x, right_x, lane_center_x)
            pub_image.publish(bridge.cv2_to_imgmsg(vis, "bgr8"))
    except Exception as e:
        rospy.logerr(f"[perception] {e}")


def main():
    global pub_center_x, pub_left_x, pub_right_x, pub_steering, pub_image
    global roi_top, roi_bottom, lane_offset, kp, kd, max_steering

    rospy.init_node('lane_perception_node')

    roi_top = rospy.get_param('~roi_top', 300)
    roi_bottom = rospy.get_param('~roi_bottom', 450)
    lane_offset = rospy.get_param('~lane_offset', 180)
    kp = rospy.get_param('~kp', 0.01)
    kd = rospy.get_param('~kd', 0.005)
    max_steering = rospy.get_param('~max_steering', 0.5)

    pub_center_x = rospy.Publisher('/limo/lane_center_x', Int32, queue_size=1)
    pub_left_x = rospy.Publisher('/limo/lane_left_x', Int32, queue_size=1)
    pub_right_x = rospy.Publisher('/limo/lane_right_x', Int32, queue_size=1)
    pub_steering = rospy.Publisher('/limo/steering_offset', Float32, queue_size=1)
    pub_image = rospy.Publisher('/limo/lane_detect/image', Image, queue_size=1)

    rospy.Subscriber(
        '/camera/color/image_raw/compressed',
        CompressedImage, image_callback,
        queue_size=1, buff_size=2**24
    )

    rospy.loginfo("[perception] started")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
