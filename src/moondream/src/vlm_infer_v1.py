#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VLM Inference Node — LIMO AutoRace
═══════════════════════════════════════════════════════════════════════════════
역할: 카메라 이미지 → VLM 질의 → 미션 후보(candidate) 발행
      거리 판단은 하지 않음. Decision Node에서 LiDAR로 게이팅.

Sub : /camera/color/image_raw/compressed  (CompressedImage)
Pub : /vlm/mission                        (String, JSON)

JSON payload
  {
    "candidate"        : str,     # 파싱된 미션 후보
    "vlm_raw"          : str,     # VLM 원문 (디버그)
    "image_stamp_sec"  : int,
    "image_stamp_nsec" : int,
    "vlm_publish_time" : float    # rospy.Time 기준
  }

Moondream2 rev 2025-01-09 API (model.query)
"""

import os
import json
import time

import rospy
import cv2
import numpy as np
import torch
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from PIL import Image as PILImage
from transformers import AutoModelForCausalLM

# ══════════════════════════════════════════════════════════════════════════════
# Globals
# ══════════════════════════════════════════════════════════════════════════════
model        = None
config       = None
mission_pub  = None
last_infer_t = 0.0


# ══════════════════════════════════════════════════════════════════════════════
# VLM Query
# ══════════════════════════════════════════════════════════════════════════════
def query_scene(encoded_image) -> str:
    result = model.query(
        encoded_image,
        config["prompt"],
        settings={"max_tokens": config.get("max_tokens", 30)},
    )
    return result["answer"].strip()


# ══════════════════════════════════════════════════════════════════════════════
# Mission Parser
# ══════════════════════════════════════════════════════════════════════════════
MISSION_RULES = [
    {
        "mission":  "cone_avoidance",
        "must_any": ["cone"],
        "must_not": [],
    },
    {
        "mission":  "crosswalk_stop",
        "must_any": ["crosswalk", "pedestrian", "zebra"],
        "must_not": ["cone"],
        "enabled":  False,       # ← 비활성 (나중에 True로 복원)
    },
    {
        "mission":  "sign_left",
        "must_any": ["sign", "arrow", "turn"],
        "must_all": ["left"],
        "must_not": ["cone", "crosswalk", "right"],
    },
    {
        "mission":  "sign_right",
        "must_any": ["sign", "arrow", "turn"],
        "must_all": ["right"],
        "must_not": ["cone", "crosswalk", "left"],
    },
]


def parse_mission(raw_text: str) -> str:
    text = raw_text.lower()
    for rule in MISSION_RULES:
        if not rule.get("enabled", True):
            continue
        if not any(kw in text for kw in rule["must_any"]):
            continue
        if rule.get("must_all") and not all(kw in text for kw in rule["must_all"]):
            continue
        if any(kw in text for kw in rule["must_not"]):
            continue
        return rule["mission"]
    return "normal_drive"


# ══════════════════════════════════════════════════════════════════════════════
# Publish
# ══════════════════════════════════════════════════════════════════════════════
def publish(candidate: str, vlm_raw: str, image_stamp):
    payload = {
        "candidate":        candidate,
        "vlm_raw":          vlm_raw,
        "image_stamp_sec":  image_stamp.secs,
        "image_stamp_nsec": image_stamp.nsecs,
        "vlm_publish_time": rospy.Time.now().to_sec(),
    }
    msg      = String()
    msg.data = json.dumps(payload)
    mission_pub.publish(msg)
    rospy.loginfo(f"[VLM] candidate={candidate:16s} | raw='{vlm_raw}'")


# ══════════════════════════════════════════════════════════════════════════════
# Image Callback
# ══════════════════════════════════════════════════════════════════════════════
def image_callback(msg: CompressedImage):
    global last_infer_t

    now = time.time()
    if now - last_infer_t < config.get("infer_interval", 0.5):
        return
    last_infer_t = now

    image_stamp = msg.header.stamp
    t0 = time.time()

    try:
        np_arr  = np.frombuffer(msg.data, np.uint8)
        cv_img  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        pil_img = PILImage.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))

        encoded  = model.encode_image(pil_img)
        vlm_raw  = query_scene(encoded)

        candidate = parse_mission(vlm_raw)

        rospy.loginfo(f"[Infer] {(time.time()-t0)*1000:.0f}ms")
        publish(candidate, vlm_raw, image_stamp)

    except Exception as e:
        rospy.logerr(f"[image_callback] {e}")
        publish("normal_drive", f"ERROR: {e}", image_stamp)


# ══════════════════════════════════════════════════════════════════════════════
# Warmup
# ══════════════════════════════════════════════════════════════════════════════
def warmup():
    rospy.loginfo("[Warmup] dummy inference...")
    dummy = PILImage.new("RGB", (640, 480), color=(128, 128, 128))
    t0 = time.time()
    enc = model.encode_image(dummy)
    model.query(enc, "What do you see?", settings={"max_tokens": 10})
    rospy.loginfo(f"[Warmup] done ({(time.time()-t0)*1000:.0f}ms)")


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════
def main():
    global model, config, mission_pub

    config_path = os.path.join(os.path.dirname(__file__), "prompt.json")
    with open(config_path) as f:
        config = json.load(f)

    rospy.init_node("vlm_infer")

    model_name = config.get("model", "vikhyatk/moondream2")
    revision   = config.get("revision", "2025-01-09")
    rospy.loginfo(f"[Init] Loading {model_name} (rev={revision}) ...")

    load_kwargs = {
        "trust_remote_code": True,
        "device_map": {"": "cuda"} if torch.cuda.is_available() else None,
    }
    if "4bit" not in model_name:
        load_kwargs["torch_dtype"] = torch.float16
    if revision:
        load_kwargs["revision"] = revision

    model = AutoModelForCausalLM.from_pretrained(model_name, **load_kwargs)
    model.eval()
    rospy.loginfo("[Init] Model loaded")

    warmup()

    mission_pub = rospy.Publisher("/vlm/mission", String, queue_size=1)
    rospy.Subscriber(
        "/camera/color/image_raw/compressed",
        CompressedImage, image_callback,
        queue_size=1, buff_size=2**24,
    )

    rospy.loginfo(f"[Init] VLM Node Ready | interval={config.get('infer_interval',0.5)}s")
    rospy.spin()


if __name__ == "__main__":
    main()