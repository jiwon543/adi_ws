#!/usr/bin/env python3
"""
decision_node.py (fixed)
VLM 힌트 (/vlm/mission JSON) + LiDAR 검증으로 미션 전환
Pub: /decision/mission (String)

수정사항:
  - VLM 미연결 시 LANE_FOLLOW 기본 퍼블리시 (차선 추종은 계속)
  - CONE_AVOID 복귀 경로 단순화 (mission_done만 사용, decision에서 자체 클리어 판정 제거)
  - verify 상태에서도 LANE_FOLLOW 퍼블리시하여 차선 이탈 방지
  - shutdown hook 추가
"""

import json

import rospy
from std_msgs.msg import String, Int32
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist

# ── 미션 / 검증 상태 상수 ────────────────────────────────────────
LANE_FOLLOW    = "LANE_FOLLOW"
CONE_AVOID     = "CONE_AVOID"
CROSSWALK      = "CROSSWALK"
EMERGENCY_STOP = "EMERGENCY_STOP"

_VRF_CONE  = "_VRF_CONE"
_VRF_EMRG  = "_VRF_EMRG"
_VRF_CROSS = "_VRF_CROSS"

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
g_yellow_count  = 0         # /perception/yellow_pixel_count

g_pub_mission   = None
g_pub_cmd       = None      # shutdown 시 정지용
g_vlm_ready     = False
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
        "cone_trigger_dist":  float(p.get("cone_trigger_dist",  1.0)),
        "cone_dist_thresh":   float(p.get("cone_dist_thresh",   1.5)),
        "cone_min_clusters":  int(p.get("cone_min_clusters",    2)),
        # emergency
        "emergency_dist":     float(p.get("emergency_dist",    0.8)),
        "emergency_y_thresh": float(p.get("emergency_y_thresh", 0.35)),
        # 미션 재진입 방지 쿨다운 [s]
        "mission_cooldown":   float(p.get("mission_cooldown",   5.0)),
        "crosswalk_cooldown": float(p.get("crosswalk_cooldown", 10.0)),
        # 전방 근접 강제 emergency (VLM 없이 LiDAR 직접 트리거)
        "lidar_emrg_force_dist": float(p.get("lidar_emrg_force_dist", 0.45)),
        "lidar_emrg_force_y":    float(p.get("lidar_emrg_force_y",   0.28)),
        # crosswalk 황색 픽셀 검증
        "yellow_thresh":         int(p.get("yellow_thresh",           800)),
    }
    rospy.loginfo("[decision] params: %s", g_p)


# ── 미션 쿨다운 관리 ────────────────────────────────────────────
_mission_done_times = {}   # {mission_name: rospy.Time}


def is_cooled_down(mission):
    """최근에 완료된 미션이면 쿨다운 기간 동안 재진입 차단"""
    if mission not in _mission_done_times:
        return True
    elapsed  = (rospy.Time.now() - _mission_done_times[mission]).to_sec()
    cooldown = g_p["crosswalk_cooldown"] if mission == CROSSWALK else g_p["mission_cooldown"]
    return elapsed > cooldown


# ── LiDAR 검증 헬퍼 ──────────────────────────────────────────────
def cone_lidar_ok():
    lat = g_p["cone_lateral_limit"]
    if g_p["cone_verify_mode"] == "distance":
        return any(0.05 < x < g_p["cone_trigger_dist"] and abs(y) < lat
                   for x, y in g_clusters)
    else:
        count = sum(1 for x, y in g_clusters
                    if 0.05 < x < g_p["cone_dist_thresh"] and abs(y) < lat)
        return count >= g_p["cone_min_clusters"]


def emergency_in_front():
    d, lat = g_p["emergency_dist"], g_p["emergency_y_thresh"]
    return any(0.05 < x < d and abs(y) < lat for x, y in g_clusters)


def lidar_emergency_close():
    """전방 근접 장애물 강제 트리거 (VLM 불필요).
    cone_lateral_limit(0.7m) 보다 훨씬 좁은 정면 폭만 체크해 콘과 구분.
    """
    d   = g_p["lidar_emrg_force_dist"]
    lat = g_p["lidar_emrg_force_y"]
    return any(0.05 < x < d and abs(y) < lat for x, y in g_clusters)


# ── 상태 전환 ────────────────────────────────────────────────────
def transition(new_state):
    global g_state, g_vrf_start, g_vlm_hint
    rospy.loginfo("[decision] %s -> %s", g_state, new_state)
    g_state     = new_state
    g_vrf_start = None
    g_vlm_hint  = None


# ── 콜백 ────────────────────────────────────────────────────────
def cb_vlm(msg):
    global g_vlm_hint, g_vlm_hint_time, g_vlm_ready
    if not g_vlm_ready:
        g_vlm_ready = True
        rospy.loginfo("[decision] VLM 연결 확인")
    try:
        j = json.loads(msg.data)
        candidate = j.get("candidate", "normal_drive")
    except (json.JSONDecodeError, AttributeError):
        rospy.logwarn("[decision] VLM JSON parse failed: %s", msg.data[:80])
        return

    mission = VLM_MAP.get(candidate, LANE_FOLLOW)
    if mission == LANE_FOLLOW:
        return

    g_vlm_hint      = mission
    g_vlm_hint_time = rospy.Time.now()
    rospy.loginfo("[decision] VLM hint: %s -> %s", candidate, mission)


def cb_clusters(msg):
    global g_clusters
    g_clusters = [(pt.x, pt.y) for pt in msg.points]


def cb_yellow(msg):
    global g_yellow_count
    g_yellow_count = msg.data


def cb_mission_done(msg):
    global g_state
    done = msg.data.strip()
    rospy.loginfo("[decision] mission_done: %s (state: %s)", done, g_state)

    if done == "CONE_AVOID_DONE" and g_state == CONE_AVOID:
        _mission_done_times[CONE_AVOID] = rospy.Time.now()
        transition(LANE_FOLLOW)
    elif done == "CROSSWALK_DONE" and g_state == CROSSWALK:
        _mission_done_times[CROSSWALK] = rospy.Time.now()
        transition(LANE_FOLLOW)
    elif done == "EMERGENCY_STOP_DONE" and g_state == EMERGENCY_STOP:
        _mission_done_times[EMERGENCY_STOP] = rospy.Time.now()
        transition(LANE_FOLLOW)


# ── FSM 루프 ─────────────────────────────────────────────────────
def decision_loop(event):
    global g_state, g_vrf_start, g_vlm_hint, g_vlm_hint_time

    now = rospy.Time.now()

    # VLM 미연결 시 정지 (lane_control이 WAIT 받으면 zero Twist 발행 후 침묵)
    if not g_vlm_ready:
        g_pub_mission.publish(String(data="WAIT"))
        rospy.logwarn_throttle(5.0, "[decision] VLM 미연결 — 정지 대기 중")
        return

    # VLM 힌트 만료
    if g_vlm_hint is not None and g_vlm_hint_time is not None:
        if (now - g_vlm_hint_time).to_sec() > g_p["vlm_hint_timeout"]:
            g_vlm_hint = None

    # LANE_FOLLOW
    if g_state == LANE_FOLLOW:
        # ① LiDAR 전방 근접 강제 트리거 (최우선 — VLM 불필요)
        #    단, VLM이 cone_avoidance 힌트 중일 때는 스킵 → 콘 구간 진입을 obstacle로 오인 방지
        if lidar_emergency_close() and is_cooled_down(EMERGENCY_STOP) \
                and g_vlm_hint != CONE_AVOID:
            rospy.logwarn("[decision] LiDAR 전방 %.2fm 이내 장애물 -> EMERGENCY_STOP 강제",
                          g_p["lidar_emrg_force_dist"])
            transition(EMERGENCY_STOP)
        # ② VLM 기반 미션 전환
        elif g_vlm_hint == CONE_AVOID and is_cooled_down(CONE_AVOID):
            g_state     = _VRF_CONE
            g_vrf_start = now
        elif g_vlm_hint == CROSSWALK and is_cooled_down(CROSSWALK):
            g_state     = _VRF_CROSS
            g_vrf_start = now
        elif g_vlm_hint == EMERGENCY_STOP and is_cooled_down(EMERGENCY_STOP):
            g_state     = _VRF_EMRG
            g_vrf_start = now

    # 검증: 콘 (VLM AND LiDAR)
    elif g_state == _VRF_CONE:
        if cone_lidar_ok():
            transition(CONE_AVOID)
        elif (now - g_vrf_start).to_sec() > g_p["verify_timeout"]:
            rospy.loginfo("[decision] CONE verify timeout -> LANE_FOLLOW")
            transition(LANE_FOLLOW)

    # 검증: Crosswalk (VLM AND 황색 픽셀)
    elif g_state == _VRF_CROSS:
        if g_yellow_count >= g_p["yellow_thresh"]:
            rospy.loginfo("[decision] yellow %d >= %d -> CROSSWALK",
                          g_yellow_count, g_p["yellow_thresh"])
            transition(CROSSWALK)
        elif (now - g_vrf_start).to_sec() > g_p["verify_timeout"]:
            rospy.loginfo("[decision] CROSSWALK verify timeout -> LANE_FOLLOW")
            transition(LANE_FOLLOW)

    # 검증: Emergency (VLM AND LiDAR)
    elif g_state == _VRF_EMRG:
        if emergency_in_front():
            transition(EMERGENCY_STOP)
        elif (now - g_vrf_start).to_sec() > g_p["verify_timeout"]:
            rospy.loginfo("[decision] EMERGENCY verify timeout -> LANE_FOLLOW")
            transition(LANE_FOLLOW)

    # CONE_AVOID — 복귀는 오직 mission_done 콜백에서만
    elif g_state == CONE_AVOID:
        pass

    # CROSSWALK — crosswalk_control이 CROSSWALK_DONE 퍼블리시
    elif g_state == CROSSWALK:
        pass

    # EMERGENCY_STOP — obstacle_stop_control이 DONE 퍼블리시
    elif g_state == EMERGENCY_STOP:
        pass

    # 검증 중에도 LANE_FOLLOW 퍼블리시 (차선 추종 유지)
    pub_state = g_state if not g_state.startswith("_VRF") else LANE_FOLLOW
    g_pub_mission.publish(String(data=pub_state))


# ── shutdown ─────────────────────────────────────────────────────
def on_shutdown():
    rospy.loginfo("[decision] shutdown — publishing zero cmd_vel")
    if g_pub_cmd is not None:
        g_pub_cmd.publish(Twist())


# ── main ─────────────────────────────────────────────────────────
def main():
    global g_pub_mission, g_pub_cmd

    rospy.init_node("decision_node")
    load_params()

    g_pub_mission = rospy.Publisher("/decision/mission", String, queue_size=1)
    g_pub_cmd     = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    rospy.Subscriber("/vlm/mission",                  String,     cb_vlm,          queue_size=1)
    rospy.Subscriber("/lidar/clusters",               PointCloud, cb_clusters,     queue_size=1)
    rospy.Subscriber("/decision/mission_done",         String,     cb_mission_done, queue_size=1)
    rospy.Subscriber("/perception/yellow_pixel_count", Int32,      cb_yellow,       queue_size=1)

    rospy.on_shutdown(on_shutdown)
    rospy.Timer(rospy.Duration(1.0 / g_p["decision_rate"]), decision_loop)

    rospy.loginfo("[decision] node ready | cone_mode=%s", g_p["cone_verify_mode"])
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
