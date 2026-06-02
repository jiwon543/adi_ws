#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
metrics.py — VLM trade-off 평가의 정확도/속도 지표 (표준 라이브러리만 사용)

- parse_mission: vlm_node.py 의 파서 로직을 그대로 미러링 (출처: src/moondream/src/vlm_node.py)
  * vlm_node 는 rospy 를 import 하므로 직접 import 불가 → 로직만 복사.
  * vlm_node.py 의 MISSION_RULES / parse_mission 을 고치면 여기도 맞춰야 함.
- 3클래스 매핑(cone/obstacle/lane) 및 Acc/Recall/Acc_bal/Acc_w/p95/혼동행렬.

설계 근거: tradeoff_eval/DESIGN.md, CONCEPT.md
"""

from collections import defaultdict

# ── 평가 클래스 (3클래스) ────────────────────────────────────────
CLASSES = ["cone", "obstacle", "lane"]

# parse_mission 출력(미션) → 평가 클래스. (DESIGN.md §4.2)
# crosswalk_stop / normal_drive 둘 다 lane(배경)으로 접는다.
MISSION_TO_CLASS = {
    "obstacle_stop":  "obstacle",
    "cone_avoidance": "cone",
    "crosswalk_stop": "lane",
    "normal_drive":   "lane",
}

# 안전 가중치 (놓치면 위험한 순). DESIGN.md §4.3
SAFETY_WEIGHTS = {"obstacle": 3.0, "cone": 2.0, "lane": 1.0}


# ── 파서 (vlm_node.py 미러) ──────────────────────────────────────
MISSION_RULES = [
    {
        "mission":  "obstacle_stop",
        "must_any": ["obstacle", "barrier", "blocked", "debris", "box",
                     "rock", "fallen", "object", "vehicle", "person",
                     "animal", "toy", "truck", "car", "van", "bus"],
        "must_not": ["cone", "cones", "traffic cone", "orange cone"],
    },
    {
        "mission":  "crosswalk_stop",
        "must_any": ["crosswalk", "crossing", "pedestrian", "zebra",
                     "yellow stripe", "yellow marking", "yellow line",
                     "striped"],
        "must_not": [],
    },
    {
        "mission":  "cone_avoidance",
        "must_any": ["cone", "cones", "construction", "detour",
                     "orange marker", "traffic cone"],
        "must_not": [],
    },
]


def parse_mission(raw_text):
    """우선순위 기반 키워드 매칭. fallback = normal_drive. (vlm_node.py 미러)"""
    text = (raw_text or "").lower().strip()

    if not text or text in ("nothing", "none", "n/a", "no"):
        return "normal_drive"

    clear_phrases = ["clear road", "empty road", "nothing ahead",
                     "no obstacle", "no hazard", "road is clear",
                     "just a road", "normal road", "open road"]
    if any(cp in text for cp in clear_phrases):
        return "normal_drive"

    for rule in MISSION_RULES:
        if not any(kw in text for kw in rule["must_any"]):
            continue
        if any(kw in text for kw in rule["must_not"]):
            continue
        return rule["mission"]

    return "normal_drive"


def predict_class(raw_text):
    """raw 답변 → 미션 → 3클래스."""
    return MISSION_TO_CLASS[parse_mission(raw_text)]


# ── 정확도 지표 ──────────────────────────────────────────────────
def recall_per_class(items):
    """
    items: iterable of (gt_class, pred_class)
    반환: {cls: recall} (해당 GT 표본이 없으면 None)
    """
    total = defaultdict(int)
    hit = defaultdict(int)
    for gt, pred in items:
        total[gt] += 1
        if pred == gt:
            hit[gt] += 1
    return {c: (hit[c] / total[c] if total[c] else None) for c in CLASSES}


def accuracy(items):
    items = list(items)
    if not items:
        return 0.0
    return sum(1 for gt, pred in items if gt == pred) / len(items)


def balanced_accuracy(items):
    """Acc_bal = 표본이 존재하는 클래스 recall 의 평균. (DESIGN.md §4.3)"""
    rec = recall_per_class(items)
    vals = [v for v in rec.values() if v is not None]
    return sum(vals) / len(vals) if vals else 0.0


def weighted_accuracy(items, weights=None):
    """Acc_w = Σ w_c·Recall_c / Σ w_c (표본 있는 클래스만)."""
    weights = weights or SAFETY_WEIGHTS
    rec = recall_per_class(items)
    num = den = 0.0
    for c, r in rec.items():
        if r is None:
            continue
        w = weights.get(c, 1.0)
        num += w * r
        den += w
    return num / den if den else 0.0


def confusion(items):
    """3x3 혼동행렬: mat[gt][pred] = count."""
    mat = {g: {p: 0 for p in CLASSES} for g in CLASSES}
    for gt, pred in items:
        mat[gt][pred] += 1
    return mat


# ── 속도 지표 ────────────────────────────────────────────────────
def percentile(values, q):
    """q in [0,100]. 선형 보간(nearest-rank 아님). values 비어있으면 0."""
    xs = sorted(values)
    if not xs:
        return 0.0
    if len(xs) == 1:
        return xs[0]
    rank = (q / 100.0) * (len(xs) - 1)
    lo = int(rank)
    hi = min(lo + 1, len(xs) - 1)
    frac = rank - lo
    return xs[lo] * (1 - frac) + xs[hi] * frac


def latency_summary(latencies):
    xs = list(latencies)
    if not xs:
        return {"n": 0, "p95": 0.0, "mean": 0.0, "max": 0.0, "min": 0.0}
    return {
        "n": len(xs),
        "p95": percentile(xs, 95),
        "mean": sum(xs) / len(xs),
        "max": max(xs),
        "min": min(xs),
    }
