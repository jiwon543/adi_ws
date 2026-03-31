#!/usr/bin/env python3
"""
lane_control.py
- lateral_offset 기반 PID 조향 + 곡률 감속
- 구독: lateral_offset, curvature, lane_detected
- 퍼블리시: /cmd_vel (Twist)
"""

import rospy
import numpy as np
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist

# ────────────────── 전역 상태 ──────────────────
g_pub_cmd = None

g_lateral   = 0.0
g_curvature = 0.0
g_detected  = False
g_last_detect_time = None

# PID 내부 상태
g_err_sum    = 0.0
g_err_prev   = 0.0
g_prev_time  = None

# 파라미터
g_p = {}


# ================================================================
#  파라미터 로드
# ================================================================
def load_params():
    global g_p
    p = rospy.get_param("~", {})

    # PID 게인 (lateral offset [px] → angular.z [rad/s])
    g_p["kp"] = float(p.get("kp", 0.005))
    g_p["ki"] = float(p.get("ki", 0.0))
    g_p["kd"] = float(p.get("kd", 0.002))

    # 적분 windup 제한
    g_p["i_max"] = float(p.get("i_max", 50.0))

    # 조향 제한
    g_p["max_angular"] = float(p.get("max_angular", 0.5))  # rad/s

    # 직진 속도
    g_p["v_base"] = float(p.get("v_base", 0.25))  # m/s

    # 곡률 감속
    g_p["v_min_curve"]      = float(p.get("v_min_curve",      0.12))
    g_p["curv_slow_thresh"] = float(p.get("curv_slow_thresh", 0.003))
    g_p["curv_slow_gain"]   = float(p.get("curv_slow_gain",   50.0))

    # 미검출 시 정지 타임아웃
    g_p["lost_timeout"] = float(p.get("lost_timeout", 1.0))

    # 제어 주기
    g_p["control_rate"] = float(p.get("control_rate", 30.0))

    rospy.loginfo("[control] params: %s", g_p)


# ================================================================
#  인지 콜백
# ================================================================
def cb_lateral(msg):
    global g_lateral
    g_lateral = msg.data

def cb_curvature(msg):
    global g_curvature
    g_curvature = msg.data

def cb_detected(msg):
    global g_detected, g_last_detect_time
    g_detected = msg.data
    if g_detected:
        g_last_detect_time = rospy.Time.now()


# ================================================================
#  PID 계산
# ================================================================
def compute_pid(error, dt):
    global g_err_sum, g_err_prev

    kp = g_p["kp"]
    ki = g_p["ki"]
    kd = g_p["kd"]

    # P
    p_term = kp * error

    # I
    g_err_sum += error * dt
    g_err_sum = np.clip(g_err_sum, -g_p["i_max"], g_p["i_max"])
    i_term = ki * g_err_sum

    # D
    d_term = 0.0
    if dt > 1e-6:
        d_term = kd * (error - g_err_prev) / dt
    g_err_prev = error

    output = p_term + i_term + d_term
    return np.clip(output, -g_p["max_angular"], g_p["max_angular"])


# ================================================================
#  속도 결정
# ================================================================
def compute_speed():
    v_base = g_p["v_base"]
    v_min  = g_p["v_min_curve"]
    thresh = g_p["curv_slow_thresh"]
    gain   = g_p["curv_slow_gain"]

    abs_curv = abs(g_curvature)
    if abs_curv > thresh:
        v = v_base - gain * (abs_curv - thresh)
    else:
        v = v_base

    return max(v, v_min)


# ================================================================
#  제어 루프
# ================================================================
def control_loop(event):
    global g_prev_time

    now = rospy.Time.now()
    dt = 0.0
    if g_prev_time is not None:
        dt = (now - g_prev_time).to_sec()
    g_prev_time = now

    cmd = Twist()

    # 미검출 → 정지
    if not g_detected:
        if g_last_detect_time is not None:
            if (now - g_last_detect_time).to_sec() > g_p["lost_timeout"]:
                g_pub_cmd.publish(cmd)
                return
        else:
            g_pub_cmd.publish(cmd)
            return

    # lateral > 0 → 우측 치우침 → angular.z > 0 (좌회전)
    steer = compute_pid(g_lateral, dt)

    cmd.linear.x  = compute_speed()
    cmd.angular.z = steer
    g_pub_cmd.publish(cmd)


# ================================================================
#  main
# ================================================================
def main():
    global g_pub_cmd

    rospy.init_node("lane_control")
    load_params()

    g_pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    rospy.Subscriber("/perception/lateral_offset", Float32, cb_lateral,   queue_size=1)
    rospy.Subscriber("/perception/curvature",      Float32, cb_curvature, queue_size=1)
    rospy.Subscriber("/perception/lane_detected",  Bool,    cb_detected,  queue_size=1)

    rate = g_p["control_rate"]
    rospy.Timer(rospy.Duration(1.0 / rate), control_loop)

    rospy.loginfo("[control] lane_control ready (rate=%.0f Hz)", rate)
    rospy.spin()


if __name__ == "__main__":
    main()
