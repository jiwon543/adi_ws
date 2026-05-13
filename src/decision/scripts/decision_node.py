#!/usr/bin/env python3
"""
decision_node.py
VLM 힌트 (/vlm/mission JSON) + LiDAR 검증으로 미션 전환
Pub: /decision/mission (String)

미션별 활성화 조건:
  CONE_AVOID     VLM AND LiDAR (distance 또는 count 모드)
  CROSSWALK      VLM 즉시 활성 → crosswalk_control이 yellow pixel 보고 정지
  EMERGENCY_STOP VLM AND LiDAR
"""

import json

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud

# ── 미션 / 검증 상태 상수 ────────────────────────────────────────
LANE_FOLLOW    = "LANE_FOLLOW"
CONE_AVOID     = "CONE_AVOID"
CROSSWALK      = "CROSSWALK"
EMERGENCY_STOP = "EMERGENCY_STOP"

_VRF_CONE = "_VRF_CONE"
_VRF_EMRG = "_VRF_EMRG"

VLM_MAP = {
    "normal_drive":   LANE_FOLLOW,
    "cone_avoidance": CONE_AVOID,
    "crosswalk_stop": CROSSWALK,
    "obstacle_stop":  EMERGENCY_STOP,
}

# ── 전역 상태 ────────────────────────────────────────────────────
g_state         = LANE_FOLLOW
g_vrf_start     = None

g_vlm_hint      = None
g_vlm_hint_time = None

g_clusters      = []        # [(x, y), ...]

g_pub_mission   = None
g_vlm_ready     = False   # VLM 첫 메시지 수신 전까지 미션 퍼블리시 차단
g_p             = {}


# ── 파라미터 로드 ────────────────────────────────────────────────
def load_params():
    global g_p
    p    = rospy.get_param("~", {})
    g_p  = {
        "vlm_hint_timeout":   float(p.get("vlm_hint_timeout",   3.0)),
        "verify_timeout":     float(p.get("verify_timeout",     2.0)),
        "decision_rate":      float(p.get("decision_rate",     10.0)),
        # cone
        "cone_verify_mode":   str(p.get("cone_verify_mode",   "distance")),
        "cone_lateral_limit": float(p.get("cone_lateral_limit", 0.8)),
        "cone_clear_dist":    float(p.get("cone_clear_dist",    2.0)),
        "cone_trigger_dist":  float(p.get("cone_trigger_dist",  1.0)),   # distance 모드
        "cone_dist_thresh":   float(p.get("cone_dist_thresh",   1.5)),   # count 모드
        "cone_min_clusters":  int(p.get("cone_min_clusters",    2)),     # count 모드
        # emergency
        "emergency_dist":     float(p.get("emergency_dist",    0.8)),
        "emergency_y_thresh": float(p.get("emergency_y_thresh", 0.35)),
    }
    rospy.loginfo("[decision] params: %s", g_p)


# ── LiDAR 검증 헬퍼 ──────────────────────────────────────────────
def cone_lidar_ok():
    lat = g_p["cone_lateral_limit"]
    if g_p["cone_verify_mode"] == "distance":
        return any(0.05 < x < g_p["cone_trigger_dist"] and abs(y) < lat
                   for x, y in g_clusters)
    else:  # count
        count = sum(1 for x, y in g_clusters
                    if 0.05 < x < g_p["cone_dist_thresh"] and abs(y) < lat)
        return count >= g_p["cone_min_clusters"]


def emergency_in_front():
    d, lat = g_p["emergency_dist"], g_p["emergency_y_thresh"]
    return any(0.05 < x < d and abs(y) < lat for x, y in g_clusters)


# ── 상태 전환 ────────────────────────────────────────────────────
def transition(new_state):
    global g_state, g_vrf_start, g_vlm_hint
    rospy.loginfo("[decision] %s → %s", g_state, new_state)
    g_state     = new_state
    g_vrf_start = None
    g_vlm_hint  = None


# ── 콜백 ────────────────────────────────────────────────────────
def cb_vlm(msg):
    global g_vlm_hint, g_vlm_hint_time, g_vlm_ready
    if not g_vlm_ready:
        g_vlm_ready = True
        rospy.loginfo("[decision] VLM 연결 확인 → 미션 활성화")
    try:
        candidate = json.loads(msg.data).get("candidate", "normal_drive")
    except (json.JSONDecodeError, AttributeError):
        rospy.logwarn("[decision] VLM JSON parse failed: %s", msg.data[:80])
        return

    mission = VLM_MAP.get(candidate, LANE_FOLLOW)
    if mission == LANE_FOLLOW:
        return

    g_vlm_hint      = mission
    g_vlm_hint_time = rospy.Time.now()
    rospy.loginfo("[decision] VLM hint: %s → %s", candidate, mission)


def cb_clusters(msg):
    global g_clusters
    g_clusters = [(pt.x, pt.y) for pt in msg.points]


def cb_mission_done(msg):
    global g_state
    done = msg.data.strip()
    rospy.loginfo("[decision] mission_done: %s (state: %s)", done, g_state)
    if done == "CONE_AVOID_DONE" and g_state == CONE_AVOID:
        transition(LANE_FOLLOW)
    elif done == "CROSSWALK_DONE" and g_state == CROSSWALK:
        transition(LANE_FOLLOW)
    elif done == "EMERGENCY_STOP_DONE" and g_state == EMERGENCY_STOP:
        transition(LANE_FOLLOW)


# ── FSM 루프 ─────────────────────────────────────────────────────
def decision_loop(event):
    global g_state, g_vrf_start, g_vlm_hint, g_vlm_hint_time

    if not g_vlm_ready:
        rospy.logwarn_throttle(5.0, "[decision] VLM 미연결 — 대기 중")
        return

    now = rospy.Time.now()

    # VLM 힌트 만료
    if g_vlm_hint is not None and g_vlm_hint_time is not None:
        if (now - g_vlm_hint_time).to_sec() > g_p["vlm_hint_timeout"]:
            g_vlm_hint = None

    # LANE_FOLLOW
    if g_state == LANE_FOLLOW:
        if g_vlm_hint == CONE_AVOID:
            g_state     = _VRF_CONE
            g_vrf_start = now
        elif g_vlm_hint == CROSSWALK:
            transition(CROSSWALK)
        elif g_vlm_hint == EMERGENCY_STOP:
            g_state     = _VRF_EMRG
            g_vrf_start = now

    # 검증: 콘 (VLM AND LiDAR)
    elif g_state == _VRF_CONE:
        if cone_lidar_ok():
            transition(CONE_AVOID)
        elif (now - g_vrf_start).to_sec() > g_p["verify_timeout"]:
            rospy.loginfo("[decision] CONE verify timeout → LANE_FOLLOW")
            transition(LANE_FOLLOW)

    # 검증: Emergency (VLM AND LiDAR)
    elif g_state == _VRF_EMRG:
        if emergency_in_front():
            transition(EMERGENCY_STOP)
        elif (now - g_vrf_start).to_sec() > g_p["verify_timeout"]:
            rospy.loginfo("[decision] EMERGENCY verify timeout → LANE_FOLLOW")
            transition(LANE_FOLLOW)

    # CONE_AVOID
    elif g_state == CONE_AVOID:
        near = [x for x, y in g_clusters if x < g_p["cone_clear_dist"]]
        if not near:
            rospy.loginfo("[decision] cones cleared → LANE_FOLLOW")
            transition(LANE_FOLLOW)

    # CROSSWALK
    elif g_state == CROSSWALK:
        pass  # crosswalk_control이 yellow pixel 감지 후 CROSSWALK_DONE 퍼블리시

    # EMERGENCY_STOP — 정지/복귀는 obstacle_stop_control이 담당
    elif g_state == EMERGENCY_STOP:
        pass

    pub_state = g_state if not g_state.startswith("_VRF") else LANE_FOLLOW
    g_pub_mission.publish(String(data=pub_state))


# ── main ─────────────────────────────────────────────────────────
def main():
    global g_pub_mission

    rospy.init_node("decision_node")
    load_params()

    g_pub_mission = rospy.Publisher("/decision/mission", String, queue_size=1)

    rospy.Subscriber("/vlm/mission",           String,     cb_vlm,          queue_size=1)
    rospy.Subscriber("/lidar/clusters",        PointCloud, cb_clusters,     queue_size=1)
    rospy.Subscriber("/decision/mission_done", String,     cb_mission_done, queue_size=1)

    rospy.Timer(rospy.Duration(1.0 / g_p["decision_rate"]), decision_loop)

    rospy.loginfo("[decision] node ready | cone_mode=%s", g_p["cone_verify_mode"])
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
