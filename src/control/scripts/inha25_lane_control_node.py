#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lane Control Node
- steering_offset + lane_speed + drive_enabled → cmd_vel 발행
"""

import rospy
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist

# ── Global State ──
steering = 0.0
speed = 0.3
#drive_enabled = True

pub_cmd = None


def steering_callback(msg):
    global steering
    steering = msg.data


def speed_callback(msg):
    global speed
    speed = msg.data

# decision node 사용할 경우 주석 해제
#def drive_enabled_callback(msg):
#    global drive_enabled
#    drive_enabled = msg.data


def timer_callback(event):
    cmd = Twist()
    if drive_enabled:
        cmd.linear.x = speed
        cmd.angular.z = -steering
    else:
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
    pub_cmd.publish(cmd)


def main():
    global pub_cmd

    rospy.init_node('lane_control_node')

    rate_hz = rospy.get_param('~control_rate', 20)

    pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rospy.Subscriber('/limo/steering_offset', Float32, steering_callback, queue_size=1)
    rospy.Subscriber('/limo/lane_speed', Float32, speed_callback, queue_size=1)
    # rospy.Subscriber('/limo/drive_enabled', Bool, drive_enabled_callback, queue_size=1)

    rospy.Timer(rospy.Duration(1.0 / rate_hz), timer_callback)

    rospy.loginfo(f"[control] started  rate={rate_hz}Hz")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
