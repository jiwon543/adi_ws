#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Obstacle Control Node
- 회피 시 gap 방향으로 조향, 평시 lane steering 사용
- decision 노드 없이도 perception + control만으로 동작 가능
  → avoid_active 관련 줄 주석 해제하면 decision 노드 연동
"""

import rospy
import numpy as np
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist

# ── Global State ──
gap_angle = 0.0
min_front_dist = 10.0
has_obstacle = False
# avoid_active = False  # decision 노드 사용 시

lane_steering = 0.0
lane_speed = 0.25
avoid_speed = 0.2
stop_distance = 0.2
max_steering = 0.5
steering_gain = 0.02

pub_cmd = None
pub_state = None


def gap_angle_callback(msg):
    global gap_angle
    gap_angle = msg.data


def front_dist_callback(msg):
    global min_front_dist
    min_front_dist = msg.data


def obstacle_callback(msg):
    global has_obstacle
    has_obstacle = msg.data


# def avoid_active_callback(msg):  # decision 노드 사용 시
#     global avoid_active
#     avoid_active = msg.data


def lane_steering_callback(msg):
    global lane_steering
    lane_steering = msg.data


def lane_speed_callback(msg):
    global lane_speed
    lane_speed = msg.data


def control_callback(event):
    cmd = Twist()

    # ── decision 노드 없이: has_obstacle 직접 사용 (히스테리시스 없음) ──
    should_avoid = has_obstacle or (min_front_dist < stop_distance)

    # ── decision 노드 사용 시: 아래로 교체 ──
    # should_avoid = avoid_active

    if min_front_dist < stop_distance:
        # 너무 가까움 → 정지
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        state = "TOO_CLOSE"
    elif should_avoid:
        # gap 방향으로 회피
        steering = np.clip(-steering_gain * gap_angle,
                           -max_steering, max_steering)
        cmd.linear.x = avoid_speed
        cmd.angular.z = steering
        state = f"AVOIDING (gap={gap_angle:.1f})"
    else:
        # 정상 → lane tracing
        cmd.linear.x = lane_speed
        cmd.angular.z = -lane_steering
        state = "LANE_TRACING"

    pub_cmd.publish(cmd)
    pub_state.publish(String(data=state))


def main():
    global avoid_speed, stop_distance, max_steering, steering_gain
    global pub_cmd, pub_state

    rospy.init_node('obstacle_control_node')

    avoid_speed = rospy.get_param('~avoid_speed', 0.2)
    stop_distance = rospy.get_param('~stop_distance', 0.2)
    max_steering = rospy.get_param('~max_steering', 0.5)
    steering_gain = rospy.get_param('~steering_gain', 0.02)

    pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub_state = rospy.Publisher('/obstacle/state', String, queue_size=1)

    # Perception 토픽
    rospy.Subscriber('/obstacle/gap_angle', Float32, gap_angle_callback, queue_size=1)
    rospy.Subscriber('/obstacle/min_front_dist', Float32, front_dist_callback, queue_size=1)
    rospy.Subscriber('/obstacle/has_obstacle', Bool, obstacle_callback, queue_size=1)

    # Decision 토픽 (decision 노드 사용 시 주석 해제)
    # rospy.Subscriber('/obstacle/avoid_active', Bool, avoid_active_callback, queue_size=1)

    # Lane 토픽
    rospy.Subscriber('/limo/steering_offset', Float32, lane_steering_callback, queue_size=1)
    rospy.Subscriber('/limo/lane_speed', Float32, lane_speed_callback, queue_size=1)

    rospy.Timer(rospy.Duration(0.05), control_callback)

    rospy.loginfo("[obs_control] started")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
