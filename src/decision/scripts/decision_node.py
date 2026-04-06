#!/usr/bin/env python3
"""
decision_node.py
- VLM 힌트 (/vlm/mission) + 센서 검증으로 미션 전환
- 미션 퍼블리시: /decision/mission
- 검증 대상: 라이다 클러스터, 노란 픽셀 카운트
- Emergency stop은 decision node 직접 /cmd_vel 제로 발행

미션 값:
  LANE_FOLLOW    기본 차선 추종
  CONE_AVOID     콘 회피
  CROSSWALK      횡단보도 정지 후 재주행
  EMERGENCY_STOP 긴급 정지

VLM 라벨 → 내부 미션 매핑 (vlm_map 파라미터로 수정 가능):
  normal_drive  → LANE_FOLLOW
  cone_avoidance→ CONE_AVOID
  crosswalk_stop→ CROSSWALK
  obstacle_stop → EMERGENCY_STOP
"""

import rospy
import numpy as np
from std_msgs.msg import String, Int32
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist

# ── 미션 상수 ──────────────────────────────────────────────────
LANE_FOLLOW    = "LANE_FOLLOW"
CONE_AVOID     = "CONE_AVOID"
CROSSWALK      = "CROSSWALK"
EMERGENCY_STOP = "EMERGENCY_STOP"

# 내부 검증 상태
_VRF_CONE  = "_VRF_CONE"
_VRF_CW    = "_VRF_CW"
_VRF_EMRG  = "_VRF_EMRG"

# ── 전역 상태 ──────────────────────────────────────────────────
g_state          = LANE_FOLLOW
g_vrf_start      = None       # 검증 시작 시각 (rospy.Time)
g_vlm_hint       = None       # 최신 VLM 힌트 (내부 미션값)
g_vlm_hint_time  = None       # VLM 힌트 수신 시각

g_clusters       = []         # [(x, y), ...] 최신 클러스터 목록
g_yellow_count   = 0          # 최신 노란 픽셀 카운트

g_pub_mission    = None
g_pub_cmd        = None       # emergency 직접 제어용

g_p              = {}

# VLM 라벨 → 내부 미션 기본 매핑
DEFAULT_VLM_MAP = {
    "normal_drive":   LANE_FOLLOW,
    "cone_avoidance": CONE_AVOID,
    "crosswalk_stop": CROSSWALK,
    "obstacle_stop":  EMERGENCY_STOP,
}


# ================================================================
#  파라미터 로드
# ================================================================
def load_params():
    global g_p
    p = rospy.get_param("~", {})

    # VLM 힌트 유효 시간 [s]
    g_p["vlm_hint_timeout"]    = float(p.get("vlm_hint_timeout",    3.0))

    # 검증 타임아웃 [s] — 이 시간 안에 센서가 확인 못하면 취소
    g_p["verify_timeout"]      = float(p.get("verify_timeout",      2.0))

    # 콘 검증
    g_p["cone_dist_thresh"]    = float(p.get("cone_dist_thresh",    1.5))   # 전방 거리 [m]
    g_p["cone_lateral_limit"]  = float(p.get("cone_lateral_limit",  0.8))   # 좌우 폭 [m]
    g_p["cone_min_clusters"]   = int(p.get("cone_min_clusters",     2))     # 최소 클러스터 수

    # 횡단보도 검증
    g_p["yellow_thresh"]       = int(p.get("yellow_thresh",         800))   # 노란 픽셀 수

    # Emergency 검증
    g_p["emergency_dist"]      = float(p.get("emergency_dist",      0.8))   # 전방 거리 [m]
    g_p["emergency_y_thresh"]  = float(p.get("emergency_y_thresh",  0.35))  # 중앙 폭 [m]
    g_p["emergency_min_count"] = int(p.get("emergency_min_count",   1))     # 클러스터 수

    # 콘 클리어 판정 (rule-based 자동 복귀)
    g_p["cone_clear_dist"]     = float(p.get("cone_clear_dist",     2.0))   # 이 거리 이상이면 없다고 판단

    # 퍼블리시 주기
    g_p["decision_rate"]       = float(p.get("decision_rate",       10.0))

    # VLM 맵 (파라미터 오버라이드 가능)
    g_p["vlm_map"]             = p.get("vlm_map", DEFAULT_VLM_MAP)

    rospy.loginfo("[decision] params: %s", g_p)


# ================================================================
#  센서 검증 헬퍼
# ================================================================
def cones_in_front():
    """전방 danger zone 내 클러스터 수 반환"""
    count = 0
    for x, y in g_clusters:
        if 0.05 < x < g_p["cone_dist_thresh"] and abs(y) < g_p["cone_lateral_limit"]:
            count += 1
    return count


def emergency_detected():
    """전방 중앙에 장애물 클러스터가 있는지"""
    for x, y in g_clusters:
        if 0.05 < x < g_p["emergency_dist"] and abs(y) < g_p["emergency_y_thresh"]:
            return True
    return False


def crosswalk_detected():
    return g_yellow_count >= g_p["yellow_thresh"]


# ================================================================
#  콜백
# ================================================================
def cb_vlm(msg: String):
    global g_vlm_hint, g_vlm_hint_time
    label = msg.data.strip().lower()
    mission = g_p["vlm_map"].get(label, LANE_FOLLOW)
    g_vlm_hint      = mission
    g_vlm_hint_time = rospy.Time.now()
    rospy.loginfo("[decision] VLM hint: %s → %s", label, mission)


def cb_clusters(msg: PointCloud):
    global g_clusters
    g_clusters = [(pt.x, pt.y) for pt in msg.points]


def cb_yellow(msg: Int32):
    global g_yellow_count
    g_yellow_count = msg.data


def cb_mission_done(msg: String):
    """제어 노드에서 미션 완료 신호"""
    global g_state
    done = msg.data.strip()
    rospy.loginfo("[decision] mission_done received: %s  (current state: %s)", done, g_state)

    if done == "CONE_AVOID_DONE" and g_state == CONE_AVOID:
        _transition(LANE_FOLLOW)
    elif done == "CROSSWALK_DONE" and g_state == CROSSWALK:
        _transition(LANE_FOLLOW)


# ================================================================
#  상태 전환
# ================================================================
def _transition(new_state: str):
    global g_state, g_vrf_start, g_vlm_hint
    rospy.loginfo("[decision] %s → %s", g_state, new_state)
    g_state     = new_state
    g_vrf_start = None
    g_vlm_hint  = None     # 힌트 소비


# ================================================================
#  메인 FSM 루프
# ================================================================
def decision_loop(event):
    global g_state, g_vrf_start, g_vlm_hint, g_vlm_hint_time

    now = rospy.Time.now()

    # VLM 힌트 만료 체크
    if g_vlm_hint is not None and g_vlm_hint_time is not None:
        if (now - g_vlm_hint_time).to_sec() > g_p["vlm_hint_timeout"]:
            g_vlm_hint = None

    # ── LANE_FOLLOW ───────────────────────────────────────────
    if g_state == LANE_FOLLOW:
        # Rule-based 우선 트리거 (VLM 없어도)
        if emergency_detected():
            _transition(EMERGENCY_STOP)
            return
        if cones_in_front() >= g_p["cone_min_clusters"]:
            _transition(CONE_AVOID)
            return

        # VLM 힌트 기반 검증 진입
        if g_vlm_hint == CONE_AVOID:
            g_state     = _VRF_CONE
            g_vrf_start = now
        elif g_vlm_hint == CROSSWALK:
            g_state     = _VRF_CW
            g_vrf_start = now
        elif g_vlm_hint == EMERGENCY_STOP:
            g_state     = _VRF_EMRG
            g_vrf_start = now

    # ── 검증: 콘 ─────────────────────────────────────────────
    elif g_state == _VRF_CONE:
        elapsed = (now - g_vrf_start).to_sec()
        if cones_in_front() >= g_p["cone_min_clusters"]:
            _transition(CONE_AVOID)
        elif elapsed > g_p["verify_timeout"]:
            rospy.loginfo("[decision] CONE verify timeout → LANE_FOLLOW")
            _transition(LANE_FOLLOW)

    # ── 검증: 횡단보도 ───────────────────────────────────────
    elif g_state == _VRF_CW:
        elapsed = (now - g_vrf_start).to_sec()
        if crosswalk_detected():
            _transition(CROSSWALK)
        elif elapsed > g_p["verify_timeout"]:
            rospy.loginfo("[decision] CROSSWALK verify timeout → LANE_FOLLOW")
            _transition(LANE_FOLLOW)

    # ── 검증: Emergency ──────────────────────────────────────
    elif g_state == _VRF_EMRG:
        elapsed = (now - g_vrf_start).to_sec()
        if emergency_detected():
            _transition(EMERGENCY_STOP)
        elif elapsed > g_p["verify_timeout"]:
            rospy.loginfo("[decision] EMERGENCY verify timeout → LANE_FOLLOW")
            _transition(LANE_FOLLOW)

    # ── CONE_AVOID ───────────────────────────────────────────
    elif g_state == CONE_AVOID:
        # rule-based 자동 복귀: 콘이 다 사라졌으면
        near = [x for x, y in g_clusters if x < g_p["cone_clear_dist"]]
        if len(near) == 0:
            rospy.loginfo("[decision] cones cleared → LANE_FOLLOW")
            _transition(LANE_FOLLOW)
        # emergency 발생 시 우선 전환
        if emergency_detected():
            _transition(EMERGENCY_STOP)

    # ── CROSSWALK ────────────────────────────────────────────
    elif g_state == CROSSWALK:
        pass  # crosswalk_control이 mission_done 퍼블리시 → cb_mission_done에서 처리

    # ── EMERGENCY_STOP ───────────────────────────────────────
    elif g_state == EMERGENCY_STOP:
        # 직접 zero cmd_vel 발행
        g_pub_cmd.publish(Twist())
        # 장애물 사라지면 복귀
        if not emergency_detected():
            rospy.loginfo("[decision] emergency cleared → LANE_FOLLOW")
            _transition(LANE_FOLLOW)
            return

    # 내부 검증 상태는 외부에 퍼블리시 안 함
    pub_state = g_state if not g_state.startswith("_VRF") else LANE_FOLLOW
    g_pub_mission.publish(String(data=pub_state))


# ================================================================
#  main
# ================================================================
def main():
    global g_pub_mission, g_pub_cmd

    rospy.init_node("decision_node")
    load_params()

    g_pub_mission = rospy.Publisher("/decision/mission",  String, queue_size=1)
    g_pub_cmd     = rospy.Publisher("/cmd_vel",           Twist,  queue_size=1)

    rospy.Subscriber("/vlm/mission",                    String,     cb_vlm,          queue_size=1)
    rospy.Subscriber("/lidar/clusters",                 PointCloud, cb_clusters,     queue_size=1)
    rospy.Subscriber("/perception/yellow_pixel_count",  Int32,      cb_yellow,       queue_size=1)
    rospy.Subscriber("/decision/mission_done",          String,     cb_mission_done, queue_size=1)

    rospy.Timer(rospy.Duration(1.0 / g_p["decision_rate"]), decision_loop)

    rospy.loginfo("[decision] node ready")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
