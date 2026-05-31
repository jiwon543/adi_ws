#!/usr/bin/env python3
"""
lane_control.py (fixed)
- Stanley 조향 제어 + 곡률 feedforward + 속도 적응
- 구독: lateral_offset, heading_error, curvature, lane_detected, /decision/mission
- 퍼블리시: /cmd_vel (Twist)

수정사항:
  - LANE_FOLLOW 아닌 미션에서 정지 cmd 발행 → 미션 전환 시 마지막 속도로 이탈하는 문제 해결
    (단, 해당 미션 컨트롤러가 자체 cmd_vel을 퍼블리시하면 그게 덮어씀 → 정상 동작)
  - shutdown hook: Ctrl+C 시 zero twist 발행
"""

import rospy
import numpy as np
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist

# ────────────────── 전역 상태 ──────────────────
g_pub_cmd = None

g_mission      = "LANE_FOLLOW"
g_mission_prev = "LANE_FOLLOW"   # 미션 전환 감지용 (zero cmd 한 번만 발행)

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

    g_p["k_heading"]  = float(p.get("k_heading",  1.0))
    g_p["k_cte"]      = float(p.get("k_cte",      0.008))
    g_p["k_ff"]       = float(p.get("k_ff",       0.5))
    g_p["k_d"]        = float(p.get("k_d",        0.0))
    g_p["v_min"]      = float(p.get("v_min",      0.1))
    g_p["max_steer"]  = float(p.get("max_steer",  0.50))
    g_p["v_base"]     = float(p.get("v_base",     0.3))
    g_p["v_min_curve"] = float(p.get("v_min_curve", 0.12))
    g_p["curv_slow_thresh"] = float(p.get("curv_slow_thresh", 0.003))
    g_p["curv_slow_gain"]   = float(p.get("curv_slow_gain",   50.0))
    g_p["lost_timeout"] = float(p.get("lost_timeout", 1.0))
    g_p["control_rate"] = float(p.get("control_rate", 30.0))
    g_p["px_to_m"]      = float(p.get("px_to_m", 0.0103))

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

def cb_mission(msg):
    global g_mission, g_mission_prev
    g_mission_prev = g_mission
    g_mission = msg.data.strip()


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

    lateral_m = lateral_px * g_p["px_to_m"]

    cte_term     = np.arctan2(k_cte * lateral_m, v)
    heading_term = k_h * heading_rad
    ff_term      = k_ff * curvature

    d_term = 0.0
    if dt > 1e-6 and k_d > 0:
        d_term = k_d * (lateral_m - g_prev_lateral) / dt
    g_prev_lateral = lateral_m

    steer = heading_term + cte_term + ff_term + d_term

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

    # LANE_FOLLOW가 아닌 경우: 미션 전환 시점에만 한 번 zero cmd 발행.
    # 이후에는 퍼블리시하지 않아야 미션 컨트롤러의 cmd_vel과 충돌하지 않음.
    if g_mission != "LANE_FOLLOW":
        if g_mission_prev == "LANE_FOLLOW":
            g_pub_cmd.publish(Twist())
        g_prev_time = None  # dt 리셋
        return

    now = rospy.Time.now()

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
                g_pub_cmd.publish(cmd)
                return
        else:
            g_pub_cmd.publish(cmd)
            return

    steer = compute_stanley_steer(g_lateral, g_heading, g_curvature, dt)
    speed = compute_speed(g_curvature)

    cmd.linear.x  = speed
    cmd.angular.z = steer
    g_pub_cmd.publish(cmd)


# ================================================================
#  shutdown
# ================================================================
def on_shutdown():
    rospy.loginfo("[control] shutdown — zero cmd_vel")
    if g_pub_cmd is not None:
        g_pub_cmd.publish(Twist())


# ================================================================
#  main
# ================================================================
def main():
    global g_pub_cmd

    rospy.init_node("lane_control")
    load_params()

    g_pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    rospy.Subscriber("/perception/lateral_offset", Float32, cb_lateral,   queue_size=1)
    rospy.Subscriber("/perception/heading_error",  Float32, cb_heading,   queue_size=1)
    rospy.Subscriber("/perception/curvature",      Float32, cb_curvature, queue_size=1)
    rospy.Subscriber("/perception/lane_detected",  Bool,    cb_detected,  queue_size=1)
    rospy.Subscriber("/decision/mission",          String,  cb_mission,   queue_size=1)

    rospy.on_shutdown(on_shutdown)

    rate = g_p["control_rate"]
    rospy.Timer(rospy.Duration(1.0 / rate), control_loop)

    rospy.loginfo("[control] lane_control ready  (rate=%.0f Hz)", rate)
    rospy.spin()


if __name__ == "__main__":
    main()