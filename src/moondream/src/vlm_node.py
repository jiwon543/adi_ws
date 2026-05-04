#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VLM Inference Node
Sub : /camera/color/image_raw/compressed  (CompressedImage)
Pub : /vlm/mission                        (String, JSON)

JSON payload:
  { "candidate": str, "vlm_raw": str, "vlm_publish_time": float }

Moondream2 rev 2025-01-09 (model.query API)
"""

import os
import json
import time

import rospy
import cv2
import numpy as np
import torch
from PIL import Image as PILImage
from transformers import AutoModelForCausalLM
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

# ── 전역 ────────────────────────────────────────────────────────
g_model       = None
g_cfg         = None
g_pub         = None
g_last_infer  = 0.0

# 트랙 배치 순서: crosswalk → cone → obstacle (먼저 매칭된 룰 우선)
MISSION_RULES = [
    {
        "mission":  "crosswalk_stop",
        "must_any": ["crosswalk", "pedestrian crossing", "zebra crossing", "crossing"],
        "must_not": ["cone"],
    },
    {
        "mission":  "cone_avoidance",
        "must_any": ["cone", "cones", "traffic cone"],
        "must_not": [],
    },
    {
        "mission":  "obstacle_stop",
        "must_any": ["car", "vehicle", "toy", "automobile", "parked", "blocking", "blocks the road"],
        "must_not": ["cone"],
    },
]


# ── 미션 파싱 ────────────────────────────────────────────────────
def parse_mission(raw_text):
    text = raw_text.lower()
    for rule in MISSION_RULES:
        if not any(kw in text for kw in rule["must_any"]):
            continue
        if any(kw in text for kw in rule["must_not"]):
            continue
        return rule["mission"]
    return "normal_drive"


# ── 이미지 전처리 ────────────────────────────────────────────────
def preprocess(msg):
    np_arr = np.frombuffer(msg.data, np.uint8)
    cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    if cv_img is None:
        return None
    h      = cv_img.shape[0]
    cv_img = cv_img[int(h * g_cfg.get("roi_top_ratio", 0.5)):, :]
    return PILImage.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))


# ── 이미지 콜백 ──────────────────────────────────────────────────
def image_cb(msg):
    global g_last_infer

    now = time.time()
    if now - g_last_infer < g_cfg.get("infer_interval", 0.5):
        return
    g_last_infer = now

    try:
        pil_img = preprocess(msg)
        if pil_img is None:
            rospy.logwarn("[VLM] imdecode failed")
            return

        enc    = g_model.encode_image(pil_img)
        result = g_model.query(enc, g_cfg["prompt"],
                               settings={"max_tokens": g_cfg.get("max_tokens", 30)})
        vlm_raw   = result["answer"].strip()
        candidate = parse_mission(vlm_raw)

        out      = String()
        out.data = json.dumps({
            "candidate":        candidate,
            "vlm_raw":          vlm_raw,
            "vlm_publish_time": rospy.Time.now().to_sec(),
        })
        g_pub.publish(out)
        rospy.loginfo("[VLM] %-16s | '%s'", candidate, vlm_raw)

    except Exception as e:
        rospy.logerr("[VLM] %s", e)


# ── 워밍업 ───────────────────────────────────────────────────────
def warmup():
    rospy.loginfo("[VLM] Warmup...")
    dummy = PILImage.new("RGB", (384, 384), color=(100, 100, 100))
    t0    = time.time()
    enc   = g_model.encode_image(dummy)
    g_model.query(enc, g_cfg["prompt"], settings={"max_tokens": 10})
    rospy.loginfo("[VLM] Warmup done (%.0f ms)", (time.time() - t0) * 1000)


# ── main ─────────────────────────────────────────────────────────
def main():
    global g_model, g_cfg, g_pub

    rospy.init_node("vlm_infer")

    config_path = rospy.get_param(
        "~config_path",
        os.path.join(os.path.dirname(__file__), "prompt_vlm_node.json"),
    )
    with open(config_path) as f:
        g_cfg = json.load(f)

    name     = g_cfg.get("model",    "vikhyatk/moondream2")
    revision = g_cfg.get("revision", "2025-01-09")
    rospy.loginfo("[VLM] Loading %s (rev=%s) ...", name, revision)

    g_model = AutoModelForCausalLM.from_pretrained(
        name,
        revision=revision,
        trust_remote_code=True,
        torch_dtype=torch.float16,
        device_map={"": "cuda"} if torch.cuda.is_available() else None,
    )
    g_model.eval()
    rospy.loginfo("[VLM] Model loaded")

    warmup()

    g_pub = rospy.Publisher("/vlm/mission", String, queue_size=1)
    rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage,
                     image_cb, queue_size=1, buff_size=2 ** 24)

    rospy.loginfo("[VLM] Ready | interval=%.1fs | roi_top=%.0f%%",
                  g_cfg.get("infer_interval", 0.5),
                  g_cfg.get("roi_top_ratio",  0.5) * 100)
    rospy.spin()


if __name__ == "__main__":
    main()
