#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
latency_limo.py — LIMO 쪽 latency 측정 노드

LIMO에서 실행.

전제:
  - LIMO와 노트북 시간은 chrony/NTP로 동기화되어 있어야 함.
  - 노트북 latency_vlm.py가 /vlm/mission JSON에 아래 값을 포함해야 함:
      image_stamp
      infer_time_ms

계산:
  total_time   = LIMO mission 수신 시간 - image_stamp
  vlm_infer    = 노트북이 보낸 infer_time_ms
  network_time = total_time - vlm_infer

즉:
  total_time = image transfer + VLM inference + mission result transfer
  network_time = total_time - VLM inference
"""

import json
import statistics
from collections import deque

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


image_times = deque(maxlen=500)
records = []


def mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def std(xs):
    if len(xs) < 2:
        return 0.0
    return statistics.stdev(xs)


def safe_float(x, default=0.0):
    try:
        return float(x)
    except Exception:
        return default


def image_callback(msg: CompressedImage):
    """
    LIMO 카메라 이미지 timestamp 저장.
    match_err 검증용으로만 사용.
    실제 total 계산은 VLM이 돌려준 image_stamp를 직접 사용함.
    """
    recv_time = rospy.Time.now().to_sec()

    header_stamp = msg.header.stamp.to_sec()
    if header_stamp <= 0:
        header_stamp = recv_time

    image_times.append({
        "header_stamp": header_stamp,
        "recv_time": recv_time,
    })


def find_nearest_image_by_header_stamp(target_stamp):
    if not image_times:
        return None

    return min(
        image_times,
        key=lambda item: abs(item["header_stamp"] - target_stamp),
    )


def mission_callback(msg: String):
    mission_recv_time = rospy.Time.now().to_sec()

    try:
        data = json.loads(msg.data)
    except Exception as e:
        rospy.logwarn(f"[LAT] JSON parse failed: {e}")
        return

    candidate = data.get("candidate", "unknown")
    vlm_raw = data.get("vlm_raw", "")

    image_stamp = safe_float(data.get("image_stamp", 0.0), 0.0)
    vlm_ms = safe_float(data.get("infer_time_ms", 0.0), 0.0)

    if image_stamp <= 0:
        rospy.logwarn(
            "[LAT] image_stamp not found. "
            "Check latency_vlm.py payload."
        )
        return

    # match_err는 timestamp 검증용.
    # latency 계산에는 직접 반영하지 않음.
    selected = find_nearest_image_by_header_stamp(image_stamp)
    if selected is not None:
        match_error_ms = abs(selected["header_stamp"] - image_stamp) * 1000.0
    else:
        match_error_ms = -1.0

    total_ms = (mission_recv_time - image_stamp) * 1000.0
    network_ms = total_ms - vlm_ms

    records.append({
        "total": total_ms,
        "vlm": vlm_ms,
        "network": network_ms,
        "match_error": match_error_ms,
    })

    totals = [r["total"] for r in records]
    vlms = [r["vlm"] for r in records]
    nets = [r["network"] for r in records]
    match_errors = [r["match_error"] for r in records if r["match_error"] >= 0]

    neg_mark = " WARNING_NEG_NET" if network_ms < 0 else ""

    rospy.loginfo(
        f"[LAT]{neg_mark} "
        f"total={total_ms:.1f}ms | "
        f"vlm_infer={vlm_ms:.1f}ms | "
        f"network={network_ms:.1f}ms | "
        f"mission={candidate} | "
        f"avg_total={mean(totals):.1f} | std_total={std(totals):.1f} | "
        f"avg_vlm={mean(vlms):.1f} | std_vlm={std(vlms):.1f} | "
        f"avg_network={mean(nets):.1f} | std_network={std(nets):.1f} | "
        f"min_network={min(nets):.1f} | max_network={max(nets):.1f} | "
        f"match_err={match_error_ms:.3f}ms | "
        f"avg_match_err={mean(match_errors):.3f} | "
        f"n={len(records)}"
    )

    if vlm_raw:
        rospy.loginfo(f"[VLM_RAW] '{vlm_raw}'")


def print_summary():
    if not records:
        rospy.loginfo("[SUMMARY] no latency records")
        return

    totals = [r["total"] for r in records]
    vlms = [r["vlm"] for r in records]
    nets = [r["network"] for r in records]
    match_errors = [r["match_error"] for r in records if r["match_error"] >= 0]

    neg_count = sum(1 for x in nets if x < 0)

    rospy.loginfo(
        f"\n{'=' * 85}\n"
        f"[LATENCY SUMMARY]\n"
        f"  n                  : {len(records)}\n"
        f"  negative_network   : {neg_count}\n"
        f"\n"
        f"  total_time         : "
        f"avg={mean(totals):8.3f}ms | "
        f"std={std(totals):8.3f}ms | "
        f"min={min(totals):8.3f}ms | "
        f"max={max(totals):8.3f}ms\n"
        f"\n"
        f"  vlm_inference_time : "
        f"avg={mean(vlms):8.3f}ms | "
        f"std={std(vlms):8.3f}ms | "
        f"min={min(vlms):8.3f}ms | "
        f"max={max(vlms):8.3f}ms\n"
        f"\n"
        f"  network_time       : "
        f"avg={mean(nets):8.3f}ms | "
        f"std={std(nets):8.3f}ms | "
        f"min={min(nets):8.3f}ms | "
        f"max={max(nets):8.3f}ms\n"
    )

    if match_errors:
        rospy.loginfo(
            f"  timestamp_match_err: "
            f"avg={mean(match_errors):8.3f}ms | "
            f"std={std(match_errors):8.3f}ms | "
            f"min={min(match_errors):8.3f}ms | "
            f"max={max(match_errors):8.3f}ms\n"
        )

    rospy.loginfo(f"{'=' * 85}")


def main():
    rospy.init_node("latency_limo", anonymous=True)

    rospy.Subscriber(
        "/camera/color/image_raw/compressed",
        CompressedImage,
        image_callback,
        queue_size=10,
        buff_size=2**24,
    )

    rospy.Subscriber(
        "/vlm/mission",
        String,
        mission_callback,
        queue_size=1,
    )

    rospy.on_shutdown(print_summary)

    rospy.loginfo("[Init] LIMO latency node ready")
    rospy.loginfo("[Init] Subscribe: /camera/color/image_raw/compressed")
    rospy.loginfo("[Init] Subscribe: /vlm/mission")
    rospy.loginfo("[Init] Metrics: total_time, vlm_inference_time, network_time")

    rospy.spin()


if __name__ == "__main__":
    main()