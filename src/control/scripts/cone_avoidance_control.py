#!/usr/bin/env python3
"""
cone_avoidance_control.py (fixed)

수정사항:
  - shutdown hook 추가
  - 나머지 로직은 기존 rev-2와 동일 (잘 동작하는 부분)
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# ── 회피 FSM 상태 ──────────────────────────────────────────────
AVOID_NONE    = 0
AVOID_REVERSE = 1
AVOID_STOP    = 2

# ── 전역 상태 ─────────────────────────────────────────────────
g_cones            = []
g_mission          = ""

g_avoid_state      = AVOID_NONE
g_avoid_left       = True
g_avoid_start_time = None

g_pub_cmd          = None
g_pub_done         = None
g_p                = {}


# ── 파라미터 로드 ──────────────────────────────────────────────
def load_params():
    global g_p
    p = rospy.get_param("~", {})

    g_p["avoid_dist"]      = float(p.get("avoid_dist",      1.5))
    g_p["lateral_limit"]   = float(p.get("lateral_limit",   0.8))
    g_p["stop_dist"]       = float(p.get("stop_dist",       0.1))
    g_p["avoid_hold_sec"]  = float(p.get("avoid_hold_sec",  0.5))
    g_p["avoid_stop_sec"]  = float(p.get("avoid_stop_sec",  1.0))
    g_p["reverse_speed"]   = float(p.get("reverse_speed",  -0.2))
    g_p["avoid_gain"]      = float(p.get("avoid_gain",      1.5))
    g_p["avoid_speed"]     = float(p.get("avoid_speed",     0.2))
    g_p["steering_bias"]   = float(p.get("steering_bias",   0.0))
    g_p["max_steering"]    = float(p.get("max_steering",    0.52))
    g_p["cone_clear_sec"]  = float(p.get("cone_clear_sec",  1.5))
    g_p["exit_y_thresh"]   = float(p.get("exit_y_thresh",   0.35))
    g_p["control_rate"]    = float(p.get("control_rate",    30.0))
    g_p["debug_mode"]      = bool(p.get("debug_mode",      False))

    rospy.loginfo("[cone_ctrl] params loaded: %s", g_p)


# ── 콘 필터링 ─────────────────────────────────────────────────
def filter_cones(cloud_points):
    result = []
    for pt in cloud_points:
        x, y = pt.x, pt.y
        if 0.05 < x < g_p["avoid_dist"] and abs(y) < g_p["lateral_limit"]:
            result.append((x, y))
    return result


def compute_gap_steering(cones):
    left_cones  = [(x, y) for x, y in cones if y >= 0]
    right_cones = [(x, y) for x, y in cones if y <  0]

    if left_cones and right_cones:
        nearest_left  = min(left_cones,  key=lambda c: c[0])
        nearest_right = min(right_cones, key=lambda c: c[0])
        gap_y = (nearest_left[1] + nearest_right[1]) / 2.0
        steering = g_p["avoid_gain"] * gap_y
    elif left_cones:
        nearest_y = min(left_cones, key=lambda c: c[0])[1]
        steering  = -g_p["avoid_gain"] * nearest_y
    else:
        nearest_y = min(right_cones, key=lambda c: c[0])[1]
        steering  = -g_p["avoid_gain"] * nearest_y

    steering += g_p["steering_bias"]
    return float(np.clip(steering, -g_p["max_steering"], g_p["max_steering"]))


# ── 콜백 ──────────────────────────────────────────────────────
def cb_clusters(msg):
    global g_cones
    g_cones = filter_cones(msg.points)


def cb_mission(msg):
    global g_mission, g_avoid_state, g_avoid_start_time
    prev = g_mission
    g_mission = msg.data.strip()
    if g_mission == "CONE_AVOID" and prev != "CONE_AVOID":
        g_avoid_state      = AVOID_NONE
        g_avoid_start_time = None
        rospy.loginfo("[cone_ctrl] mission activated")


_cone_clear_since = None


# ── 제어 루프 ─────────────────────────────────────────────────
def control_loop(event):
    global g_avoid_state, g_avoid_left, g_avoid_start_time
    global _cone_clear_since

    if not g_p["debug_mode"] and g_mission != "CONE_AVOID":
        return

    now   = rospy.Time.now()
    cmd   = Twist()
    cones = list(g_cones)

    # 1) 회피 FSM 트리거
    if g_avoid_state == AVOID_NONE:
        close_left  = [c for c in cones if c[0] < g_p["stop_dist"] and c[1] >= 0]
        close_right = [c for c in cones if c[0] < g_p["stop_dist"] and c[1] <  0]

        if close_left or close_right:
            g_avoid_left       = bool(close_left)
            g_avoid_state      = AVOID_REVERSE
            g_avoid_start_time = now
            rospy.logwarn("[cone_ctrl] AVOID triggered — dir=%s",
                          "LEFT" if g_avoid_left else "RIGHT")

    # 2) AVOID_REVERSE
    if g_avoid_state == AVOID_REVERSE:
        elapsed = (now - g_avoid_start_time).to_sec()
        if elapsed < g_p["avoid_hold_sec"]:
            cmd.linear.x  = g_p["reverse_speed"]
            cmd.angular.z = -g_p["max_steering"] if g_avoid_left else g_p["max_steering"]
        else:
            g_avoid_state      = AVOID_STOP
            g_avoid_start_time = now
        g_pub_cmd.publish(cmd)
        return

    # 3) AVOID_STOP
    if g_avoid_state == AVOID_STOP:
        elapsed = (now - g_avoid_start_time).to_sec()
        if elapsed < g_p["avoid_stop_sec"]:
            cmd.linear.x  = 0.0
            cmd.angular.z = -g_p["max_steering"] if g_avoid_left else g_p["max_steering"]
        else:
            g_avoid_state = AVOID_NONE
        g_pub_cmd.publish(cmd)
        return

    # 4) 원거리 콘: gap 중심 조향
    if cones:
        steering      = compute_gap_steering(cones)
        cmd.linear.x  = g_p["avoid_speed"]
        cmd.angular.z = steering
        g_pub_cmd.publish(cmd)

    # 5) 클리어 판정: 정면 경로(exit_y_thresh 이내)에 콘 없으면 타이머 시작
    path_cones = [c for c in cones if abs(c[1]) < g_p["exit_y_thresh"]]
    if path_cones:
        _cone_clear_since = None
    else:
        if _cone_clear_since is None:
            _cone_clear_since = now
        elif (now - _cone_clear_since).to_sec() >= g_p["cone_clear_sec"]:
            rospy.loginfo("[cone_ctrl] path clear -> CONE_AVOID_DONE")
            g_pub_done.publish(String(data="CONE_AVOID_DONE"))
            _cone_clear_since = None

    if not cones:
        cmd.linear.x  = g_p["avoid_speed"]
        cmd.angular.z = 0.0
        g_pub_cmd.publish(cmd)


# ── shutdown ─────────────────────────────────────────────────
def on_shutdown():
    rospy.loginfo("[cone_ctrl] shutdown — zero cmd_vel")
    if g_pub_cmd is not None:
        g_pub_cmd.publish(Twist())


# ── main ──────────────────────────────────────────────────────
def main():
    global g_pub_cmd, g_pub_done

    rospy.init_node("cone_avoidance_control")
    load_params()

    g_pub_cmd  = rospy.Publisher("/cmd_vel",               Twist,  queue_size=1)
    g_pub_done = rospy.Publisher("/decision/mission_done", String, queue_size=1)

    rospy.Subscriber("/lidar/clusters",   PointCloud, cb_clusters, queue_size=1)
    rospy.Subscriber("/decision/mission", String,     cb_mission,  queue_size=1)

    rospy.on_shutdown(on_shutdown)
    rospy.Timer(rospy.Duration(1.0 / g_p["control_rate"]), control_loop)

    rospy.loginfo("[cone_ctrl] node started (rate=%.1f Hz)", g_p["control_rate"])
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass