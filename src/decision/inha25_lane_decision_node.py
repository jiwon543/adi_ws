#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lane Decision Node
- stop_flag, parking 상태만 반영
- drive_enabled, lane_speed publish
"""

import rospy
from std_msgs.msg import Float32, Bool, String

# ── Global State ──
stop_flag = False
parking_active = False
base_speed = 0.3

pub_drive_enabled = None
pub_speed = None


def stop_callback(msg):
    global stop_flag
    stop_flag = msg.data


def parking_callback(msg):
    global parking_active
    if msg.data == "DONE":
        parking_active = True
        rospy.loginfo("[decision] Parking complete - drive disabled permanently")
    else:
        parking_active = (msg.data != "IDLE")


def timer_callback(event):
    drive_enabled = not stop_flag and not parking_active
    pub_drive_enabled.publish(Bool(data=drive_enabled))
    pub_speed.publish(Float32(data=base_speed))


def main():
    global pub_drive_enabled, pub_speed, base_speed

    rospy.init_node('lane_decision_node')

    base_speed = rospy.get_param('~base_speed', 0.3)

    pub_drive_enabled = rospy.Publisher('/limo/drive_enabled', Bool, queue_size=1)
    pub_speed = rospy.Publisher('/limo/lane_speed', Float32, queue_size=1)

    rospy.Subscriber('/limo/traffic_stop', Bool, stop_callback, queue_size=1)
    rospy.Subscriber('/parking/state', String, parking_callback, queue_size=1)

    rospy.Timer(rospy.Duration(0.1), timer_callback)

    rospy.loginfo("[decision] started")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
