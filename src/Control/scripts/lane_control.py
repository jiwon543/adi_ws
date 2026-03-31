#!/usr/bin/env python3
"""
lane_control.py
- Stanley 조향 제어 + 곡률 feedforward + 속도 적응
- 구독: lateral_offset, heading_error, curvature, lane_detected
- 퍼블리시: /cmd_vel (Twist)
-
- Stanley 공식:
-   δ = θ_heading + arctan(K_cte * e_lateral / v)
- + feedforward: K_ff * curvature
- + 속도: 곡률 클수록 감속
"""

import rospy
import numpy as np
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist

# ────────────────── 전역 상태 ──────────────────
g_pub_cmd = None

# 최신 인지 값
g_lateral   = 0.0
g_heading   = 0.0
g_curvature = 0.0
g_detected  = False
g_last_detect_time = None

# 이전 lateral (미분항용)
g_prev_lateral = 0.0
g_prev_time    = None

# 파라미터
g_p = {}


# ================================================================
#  파라미터 로드
# ================================================================
def load_params():
    global g_p
    p = rospy.get_param("~", {})

    # Stanley gains
    g_p["k_heading"]  = float(p.get("k_heading",  1.0))   # heading error 게인
    g_p["k_cte"]      = float(p.get("k_cte",      0.008)) # cross-track 게인 (px 단위이므로 작게)
    g_p["k_ff"]       = float(p.get("k_ff",        0.5))   # curvature feedforward 게인
    g_p["k_d"]        = float(p.get("k_d",         0.0))   # lateral 미분 게인 (옵션)

    # softening 속도 (Stanley arctan 분모, 0 나눗셈 방지)
    g_p["v_min"]      = float(p.get("v_min",      0.1))   # m/s

    # 조향 제한
    g_p["max_steer"]  = float(p.get("max_steer",  0.50))  # rad

    # 속도 제어
    g_p["v_base"]     = float(p.get("v_base",     0.3))  # m/s 기본 속도
    g_p["v_min_curve"] = float(p.get("v_min_curve", 0.12)) # 급커브 최저 속도
    g_p["curv_slow_thresh"] = float(p.get("curv_slow_thresh", 0.003))  # 감속 시작 곡률
    g_p["curv_slow_gain"]   = float(p.get("curv_slow_gain",   50.0))   # 곡률→감속 비례 계수

    # 미검출 시 정지 타임아웃 (초)
    g_p["lost_timeout"] = float(p.get("lost_timeout", 1.0))

    # 제어 주기 (Hz)
    g_p["control_rate"] = float(p.get("control_rate", 30.0))

    # lateral offset → meter 변환 계수 (BEV px → 실제 m, 캘리브 필요)
    # 일단 px 기반으로 동작하되, 추후 보정 가능
    g_p["px_to_m"] = float(p.get("px_to_m", 1.0))  # 1이면 px 그대로

    rospy.loginfo("[control] params loaded: %s", g_p)


# ================================================================
#  인지 콜백
# ================================================================
def cb_lateral(msg):
    global g_lateral
    g_lateral = msg.data

def cb_heading(msg):
    global g_heading
    g_heading = msg.data

def cb_curvature(msg):
    global g_curvature
    g_curvature = msg.data

def cb_detected(msg):
    global g_detected, g_last_detect_time
    g_detected = msg.data
    if g_detected:
        g_last_detect_time = rospy.Time.now()


# ================================================================
#  Stanley 조향 계산
# ================================================================
def compute_stanley_steer(lateral_px, heading_rad, curvature, dt):
    global g_prev_lateral

    k_h   = g_p["k_heading"]
    k_cte = g_p["k_cte"]
    k_ff  = g_p["k_ff"]
    k_d   = g_p["k_d"]
    v_min = g_p["v_min"]
    v     = max(g_p["v_base"], v_min)

    # Stanley: δ = k_h * θ + arctan(k_cte * e / v)
    cte_term = np.arctan2(k_cte * lateral_px, v)
    heading_term = k_h * heading_rad
    ff_term = k_ff * curvature

    # 미분항 (lateral 변화율)
    d_term = 0.0
    if dt > 1e-6 and k_d > 0:
        d_term = k_d * (lateral_px - g_prev_lateral) / dt
    g_prev_lateral = lateral_px

    steer = heading_term + cte_term + ff_term + d_term

    # 제한
    max_s = g_p["max_steer"]
    steer = np.clip(steer, -max_s, max_s)
    return steer


# ================================================================
#  속도 결정 (곡률 기반 감속)
# ================================================================
def compute_speed(curvature):
    v_base = g_p["v_base"]
    v_min  = g_p["v_min_curve"]
    thresh = g_p["curv_slow_thresh"]
    gain   = g_p["curv_slow_gain"]

    abs_curv = abs(curvature)
    if abs_curv > thresh:
        reduction = gain * (abs_curv - thresh)
        v = v_base - reduction
    else:
        v = v_base

    return max(v, v_min)


# ================================================================
#  제어 루프 (타이머 콜백)
# ================================================================
def control_loop(event):
    global g_prev_time

    now = rospy.Time.now()

    # dt 계산
    dt = 0.0
    if g_prev_time is not None:
        dt = (now - g_prev_time).to_sec()
    g_prev_time = now

    cmd = Twist()

    # 차선 미검출 타임아웃
    if not g_detected:
        if g_last_detect_time is not None:
            elapsed = (now - g_last_detect_time).to_sec()
            if elapsed > g_p["lost_timeout"]:
                # 정지
                g_pub_cmd.publish(cmd)
                return
        else:
            # 한 번도 검출 안 됨
            g_pub_cmd.publish(cmd)
            return

    # Stanley 조향
    steer = compute_stanley_steer(g_lateral, g_heading, g_curvature, dt)

    # 속도
    speed = compute_speed(g_curvature)

    cmd.linear.x  = speed
    cmd.angular.z = steer
    g_pub_cmd.publish(cmd)


# ================================================================
#  main
# ================================================================
def main():
    global g_pub_cmd

    rospy.init_node("lane_control")
    load_params()

    # 퍼블리셔
    g_pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # 서브스크라이버
    rospy.Subscriber("/perception/lateral_offset", Float32, cb_lateral,   queue_size=1)
    rospy.Subscriber("/perception/heading_error",  Float32, cb_heading,   queue_size=1)
    rospy.Subscriber("/perception/curvature",      Float32, cb_curvature, queue_size=1)
    rospy.Subscriber("/perception/lane_detected",  Bool,    cb_detected,  queue_size=1)

    # 제어 타이머
    rate = g_p["control_rate"]
    rospy.Timer(rospy.Duration(1.0 / rate), control_loop)

    rospy.loginfo("[control] lane_control ready  (rate=%.0f Hz)", rate)
    rospy.spin()


if __name__ == "__main__":
    main()