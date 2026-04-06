#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Obstacle Decision Node
- has_obstacle, min_front_dist 기반 상태 머신
- 히스테리시스(clear_threshold)로 안정적 전환
- avoid_active (Bool), obstacle_state (String) publish
"""

import rospy
from std_msgs.msg import Float32, Bool, String

# ── Global State ──
has_obstacle = False
min_front_dist = 10.0
stop_distance = 0.2
clear_threshold = 20

avoiding = False
clear_count = 0
last_state = ""

pub_avoid_active = None
pub_state = None


def obstacle_callback(msg):
    global has_obstacle
    has_obstacle = msg.data


def front_dist_callback(msg):
    global min_front_dist
    min_front_dist = msg.data


def timer_callback(event):
    global avoiding, clear_count, last_state

    # TOO_CLOSE
    if min_front_dist < stop_distance:
        avoiding = True
        clear_count = 0
        state = "TOO_CLOSE"
        avoid_active = True

    # 장애물 감지 → 회피 진입
    elif has_obstacle:
        avoiding = True
        clear_count = 0
        state = "AVOIDING"
        avoid_active = True

    # 회피 중 → 히스테리시스
    elif avoiding:
        clear_count += 1
        if clear_count >= clear_threshold:
            avoiding = False
            clear_count = 0
            state = "LANE_TRACING"
            avoid_active = False
        else:
            state = f"AVOIDING_CLEAR ({clear_count}/{clear_threshold})"
            avoid_active = True

    # 정상 주행
    else:
        state = "LANE_TRACING"
        avoid_active = False

    if state != last_state:
        rospy.loginfo(f"[obs_decision] {state}")
        last_state = state

    pub_avoid_active.publish(Bool(data=avoid_active))
    pub_state.publish(String(data=state))


def main():
    global stop_distance, clear_threshold
    global pub_avoid_active, pub_state

    rospy.init_node('obstacle_decision_node')

    stop_distance = rospy.get_param('~stop_distance', 0.2)
    clear_threshold = rospy.get_param('~clear_threshold', 20)

    pub_avoid_active = rospy.Publisher('/obstacle/avoid_active', Bool, queue_size=1)
    pub_state = rospy.Publisher('/obstacle/state', String, queue_size=1)

    rospy.Subscriber('/obstacle/has_obstacle', Bool, obstacle_callback, queue_size=1)
    rospy.Subscriber('/obstacle/min_front_dist', Float32, front_dist_callback, queue_size=1)

    rospy.Timer(rospy.Duration(0.05), timer_callback)

    rospy.loginfo(f"[obs_decision] started  stop={stop_distance}m clear={clear_threshold}")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
