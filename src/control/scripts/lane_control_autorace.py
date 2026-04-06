#!/usr/bin/env python3
"""
lane_control.py
- lateral_offset 기반 PID 조향 + 곡률 감속
- 구독: lateral_offset, curvature, lane_detected
- 퍼블리시: /cmd_vel (Twist)
- 부호: angular.z = -steering (LIMO Pro 기준)
"""

import rospy
import numpy as np
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist

# ────────────────── 전역 상태 ──────────────────
g_pub_cmd   = None
g_pub_steer = None

g_lateral   = 0.0
g_curvature = 0.0
g_detected  = False
g_last_detect_time = None

# PID 내부 상태
g_err_prev   = 0.0
g_err_sum    = 0.0

# 외부 플래그
g_stop_flag      = False
g_parking_active = False

# 파라미터
g_p = {}


# ================================================================
#  파라미터 로드
# ================================================================
def load_params():
    global g_p
    p = rospy.get_param("~", {})

    # PID 게인 (lateral offset [px] → steering)
    g_p["kp"] = float(p.get("kp", 0.01))
    g_p["ki"] = float(p.get("ki", 0.0))
    g_p["kd"] = float(p.get("kd", 0.005))
    g_p["i_max"] = float(p.get("i_max", 50.0))  # 적분 windup 제한

    # 조향 제한
    g_p["max_steering"] = float(p.get("max_steering", 0.5))

    # 직진 속도
    g_p["base_speed"] = float(p.get("base_speed", 0.3))

    # 곡률 감속
    g_p["v_min_curve"]      = float(p.get("v_min_curve",      0.12))
    g_p["curv_slow_thresh"] = float(p.get("curv_slow_thresh", 0.003))
    g_p["curv_slow_gain"]   = float(p.get("curv_slow_gain",   50.0))

    # 미검출 시 정지 타임아웃
    g_p["lost_timeout"] = float(p.get("lost_timeout", 1.0))

    # 제어 주기
    g_p["control_rate"] = float(p.get("control_rate", 30.0))

    rospy.loginfo("[control] kp=%.4f  ki=%.4f  kd=%.4f  base_speed=%.2f",
                  g_p["kp"], g_p["ki"], g_p["kd"], g_p["base_speed"])


# ================================================================
#  인지 / 외부 콜백
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

# inha2025에서 나온 detection node 살려놨음
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
#  속도 결정 (곡률 감속)
# ================================================================
def compute_speed():
    v_base = g_p["base_speed"]
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
#  제어 루프 (타이머)
# ================================================================
def control_loop(event):
    global g_err_prev, g_err_sum

    cmd = Twist()

    # 주차 중이면 발행 안 함
    if g_parking_active:
        return

    # 정지 플래그
    if g_stop_flag:
        g_pub_cmd.publish(cmd)
        return

    # 미검출 → 타임아웃 후 정지
    if not g_detected:
        now = rospy.Time.now()
        if g_last_detect_time is not None:
            if (now - g_last_detect_time).to_sec() > g_p["lost_timeout"]:
                g_pub_cmd.publish(cmd)
                return
        else:
            g_pub_cmd.publish(cmd)
            return

    # PID (프레임 기반 미분 — dt 나누지 않음)
    error = g_lateral
    derivative = error - g_err_prev

    # I항 (프레임 단위 누적)
    g_err_sum += error
    g_err_sum = np.clip(g_err_sum, -g_p["i_max"], g_p["i_max"])

    steering = np.clip(
        g_p["kp"] * error + g_p["ki"] * g_err_sum + g_p["kd"] * derivative,
        -g_p["max_steering"],
         g_p["max_steering"]
    )
    g_err_prev = error

    g_pub_steer.publish(Float32(data=steering))

    cmd.linear.x  = compute_speed()
    cmd.angular.z = -steering
    g_pub_cmd.publish(cmd)


# ================================================================
#  main
# ================================================================
def main():
    global g_pub_cmd, g_pub_steer

    rospy.init_node("lane_control")
    load_params()

    g_pub_cmd   = rospy.Publisher("/cmd_vel",              Twist,   queue_size=1)
    g_pub_steer = rospy.Publisher("/limo/steering_offset", Float32, queue_size=1)

    rospy.Subscriber("/perception/lateral_offset", Float32, cb_lateral,   queue_size=1)
    rospy.Subscriber("/perception/curvature",      Float32, cb_curvature, queue_size=1)
    rospy.Subscriber("/perception/lane_detected",  Bool,    cb_detected,  queue_size=1)
    rospy.Subscriber("/limo/traffic_stop",         Bool,    cb_stop,      queue_size=1)
    rospy.Subscriber("/parking/state",             String,  cb_parking,   queue_size=1)

    rate = g_p["control_rate"]
    rospy.Timer(rospy.Duration(1.0 / rate), control_loop)

    rospy.loginfo("[control] lane_control ready (rate=%.0f Hz)", rate)
    rospy.spin()


if __name__ == "__main__":
    main()
