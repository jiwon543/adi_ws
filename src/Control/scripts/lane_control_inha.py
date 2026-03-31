#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lane_control.py
- 인지 노드에서 lateral_offset 받아서 PD 조향
- 동작 검증된 LaneDetectNode 제어부 그대로 추출
"""

import rospy
import numpy as np
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist

# ────────────────── 전역 상태 ──────────────────
g_pub_cmd   = None
g_pub_steer = None

g_lateral    = 0.0
g_prev_error = 0.0
g_stop_flag  = False
g_parking_active = False

# 파라미터
g_p = {}


# ================================================================
#  파라미터 로드
# ================================================================
def load_params():
    global g_p
    g_p["base_speed"]    = rospy.get_param("~base_speed",    0.3)
    g_p["kp"]            = rospy.get_param("~kp",            0.01)
    g_p["kd"]            = rospy.get_param("~kd",            0.005)
    g_p["max_steering"]  = rospy.get_param("~max_steering",  0.5)
    rospy.loginfo("[control] kp=%.4f  kd=%.4f  base_speed=%.2f",
                  g_p["kp"], g_p["kd"], g_p["base_speed"])


# ================================================================
#  콜백
# ================================================================
def cb_lateral(msg):
    global g_lateral
    g_lateral = msg.data

def cb_stop(msg):
    global g_stop_flag
    g_stop_flag = msg.data

def cb_parking(msg):
    global g_parking_active
    if msg.data == "DONE":
        g_parking_active = True
        rospy.loginfo("[control] Parking complete - lane control stopped")
    else:
        g_parking_active = (msg.data != "IDLE")


# ================================================================
#  제어 콜백 (lateral_offset 수신 시마다 실행)
# ================================================================
def cb_control(msg):
    global g_prev_error

    error = msg.data  # lateral_offset (px), 양수 = 우측 치우침

    # PD (프레임 기반 미분, dt 없이)
    derivative = error - g_prev_error
    steering = np.clip(
        g_p["kp"] * error + g_p["kd"] * derivative,
        -g_p["max_steering"],
         g_p["max_steering"]
    )
    g_prev_error = error

    # steering 퍼블리시 (다른 노드 참조용)
    g_pub_steer.publish(Float32(data=steering))

    # 주차 중이면 cmd_vel 발행 안 함
    if g_parking_active:
        return

    cmd = Twist()
    if g_stop_flag:
        cmd.linear.x  = 0.0
        cmd.angular.z = 0.0
    else:
        cmd.linear.x  = g_p["base_speed"]
        cmd.angular.z = -steering
    g_pub_cmd.publish(cmd)


# ================================================================
#  main
# ================================================================
def main():
    global g_pub_cmd, g_pub_steer

    rospy.init_node("lane_control")
    load_params()

    # 퍼블리셔
    g_pub_cmd   = rospy.Publisher("/cmd_vel",              Twist,   queue_size=1)
    g_pub_steer = rospy.Publisher("/limo/steering_offset", Float32, queue_size=1)

    # 서브스크라이버
    rospy.Subscriber("/perception/lateral_offset", Float32, cb_control,  queue_size=1)
    rospy.Subscriber("/limo/traffic_stop",         Bool,    cb_stop,     queue_size=1)
    rospy.Subscriber("/parking/state",             String,  cb_parking,  queue_size=1)

    rospy.loginfo("[control] lane_control ready")
    rospy.spin()


if __name__ == "__main__":
    main()
