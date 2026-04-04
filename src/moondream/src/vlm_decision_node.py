#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Decision Node — Latency Checker (측정 부분만 발췌)

/vlm/mission JSON payload를 수신해서
  ① 구간별 레이턴시 출력
  ② fallback 분기

전체 사이클 정의
  image_stamp          → 카메라가 프레임을 캡처한 시각 (ROS header.stamp)
  vlm_publish_time     → VLM 노드가 토픽을 발행하기 직전 시각 (rospy.Time)
  t_recv               → Decision 노드가 콜백 진입한 시각 (rospy.Time)

  infer_latency  = vlm_publish_time - image_stamp   (캡처 → VLM 발행)
  topic_latency  = t_recv - vlm_publish_time         (VLM 발행 → Decision 수신)
  total_latency  = t_recv - image_stamp              (전체 사이클)

변경 이력 (v2)
  - time.time() → rospy.Time.now().to_sec()  (rosmaster 기준 동기화)
"""

import json

import rospy
from std_msgs.msg import String


# ══════════════════════════════════════════════════════════════════════════════
# Mission Callback
# ══════════════════════════════════════════════════════════════════════════════
def mission_callback(msg: String):
    t_recv = rospy.Time.now().to_sec()          # ← ROS 시간 기준

    try:
        data = json.loads(msg.data)
    except json.JSONDecodeError as e:
        rospy.logerr(f"[Decision] JSON parse error: {e}")
        return

    # ── Latency 계산 ──────────────────────────────────────────────────────
    image_stamp = data["image_stamp_sec"] + data["image_stamp_nsec"] * 1e-9
    vlm_pub     = data["vlm_publish_time"]

    infer_ms = (vlm_pub  - image_stamp) * 1000   # 캡처 → VLM 발행
    topic_ms = (t_recv   - vlm_pub    ) * 1000   # VLM 발행 → Decision 수신
    total_ms = (t_recv   - image_stamp) * 1000   # 전체 사이클

    rospy.loginfo(
        f"[Latency] infer={infer_ms:6.1f}ms | "
        f"topic={topic_ms:5.1f}ms | "
        f"total={total_ms:6.1f}ms"
    )

    # ── Mission / Fallback 분기 ───────────────────────────────────────────
    mission    = data.get("mission",    "normal_drive")
    confidence = data.get("confidence", 0.0)
    fallback   = data.get("fallback",   True)
    vlm_raw    = data.get("vlm_raw",    "")

    if fallback:
        rospy.logwarn(
            f"[Decision] FALLBACK (conf={confidence:.3f}) → rule-base only | raw='{vlm_raw}'"
        )
        run_rule_based()
        return

    rospy.loginfo(f"[Decision] mission={mission} conf={confidence:.3f}")
    handle_mission(mission)


# ══════════════════════════════════════════════════════════════════════════════
# Stub handlers  (실제 구현으로 교체)
# ══════════════════════════════════════════════════════════════════════════════
def run_rule_based():
    """VLM 신뢰 불가 → 로컬 퍼셉션(LiDAR/카메라 픽셀)만으로 판단."""
    pass


def handle_mission(mission: str):
    """
    mission 값에 따라 state 전환.
    Entry는 여기서, Exit은 각 미션 내부 퍼셉션이 담당.
    """
    dispatch = {
        "normal_drive":   on_normal_drive,
        "cone_avoidance": on_cone_avoidance,
        "crosswalk_stop": on_crosswalk_stop,
        "sign_left":      on_sign_left,
        "sign_right":     on_sign_right,
    }
    dispatch.get(mission, on_normal_drive)()


def on_normal_drive():   pass
def on_cone_avoidance(): pass
def on_crosswalk_stop(): pass
def on_sign_left():      pass
def on_sign_right():     pass


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════
def main():
    rospy.init_node("decision_node")
    rospy.Subscriber("/vlm/mission", String, mission_callback, queue_size=1)
    rospy.loginfo("[Decision] Node Ready")
    rospy.spin()


if __name__ == "__main__":
    main()
