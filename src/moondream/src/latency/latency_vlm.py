#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
latency_vlm.py — VLM 로컬 추론 + normal_drive 고정 publish

노트북에서 실행.

역할:
  1) /camera/color/image_raw/compressed subscribe
  2) Moondream VLM inference 수행
  3) VLM raw 결과와 latency는 로그/측정용으로 유지
  4) /vlm/mission의 candidate는 무조건 normal_drive로 publish

LIMO latency_limo.py에서 사용하는 값:
  image_stamp   : 원본 camera msg.header.stamp
  infer_time_ms : VLM 추론 시간
  candidate     : 항상 normal_drive
"""

import os
import json
import statistics

import rospy
import cv2
import numpy as np
import torch

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from PIL import Image as PILImage
from transformers import AutoModelForCausalLM


model = None
config = None
mission_pub = None

last_infer_t = 0.0
vlm_times = []


def mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def std(xs):
    if len(xs) < 2:
        return 0.0
    return statistics.stdev(xs)


def image_callback(msg: CompressedImage):
    global last_infer_t

    now_sec = rospy.Time.now().to_sec()

    infer_interval = float(config.get("infer_interval", 0.5))
    if now_sec - last_infer_t < infer_interval:
        return
    last_infer_t = now_sec

    image_stamp = msg.header.stamp.to_sec()
    if image_stamp <= 0:
        image_stamp = now_sec

    try:
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_img is None:
            rospy.logwarn("[VLM] cv2.imdecode failed")
            return

        # VRAM 절약
        input_size = int(config.get("input_size", 384))
        cv_img = cv2.resize(cv_img, (input_size, input_size))

        pil_img = PILImage.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))

        t0 = rospy.Time.now()

        with torch.inference_mode():
            encoded = model.encode_image(pil_img)

            result = model.query(
                encoded,
                config["prompt"],
                settings={"max_tokens": config.get("max_new_tokens", 30)},
            )

        t1 = rospy.Time.now()

        vlm_raw = result["answer"].strip()
        vlm_ms = (t1 - t0).to_sec() * 1000.0
        vlm_times.append(vlm_ms)

        # 핵심: 추론 결과와 상관없이 mission candidate는 normal_drive 고정
        candidate = "normal_drive"

        rospy.loginfo(
            f"[VLM] "
            f"vlm={vlm_ms:.1f}ms | "
            f"avg={mean(vlm_times):.1f}ms | "
            f"std={std(vlm_times):.1f}ms | "
            f"min={min(vlm_times):.1f}ms | "
            f"max={max(vlm_times):.1f}ms | "
            f"n={len(vlm_times)} | "
            f"mission={candidate} | "
        )

        payload = json.dumps({
            "candidate": candidate,
            "image_stamp": image_stamp,
            "infer_time_ms": round(vlm_ms, 1),
        })

        out_msg = String()
        out_msg.data = payload
        mission_pub.publish(out_msg)

    except torch.cuda.OutOfMemoryError as e:
        rospy.logerr(f"[VLM] CUDA OOM: {e}")
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

    except Exception as e:
        rospy.logerr(f"[VLM] {e}")


def print_summary():
    if not vlm_times:
        rospy.loginfo("[VLM SUMMARY] no inference records")
        return

    rospy.loginfo(
        f"\n{'=' * 65}\n"
        f"[VLM SUMMARY]\n"
        f"  n                  : {len(vlm_times)}\n"
        f"  vlm_inference_time : "
        f"avg={mean(vlm_times):8.3f}ms | "
        f"std={std(vlm_times):8.3f}ms | "
        f"min={min(vlm_times):8.3f}ms | "
        f"max={max(vlm_times):8.3f}ms\n"
        f"{'=' * 65}"
    )


def load_model():
    global model

    model_name = config.get("model", "vikhyatk/moondream2")
    revision = config.get("revision", "2025-01-09")

    rospy.loginfo(f"[Init] Loading {model_name} (rev={revision}) ...")
    rospy.loginfo(f"[Init] CUDA available: {torch.cuda.is_available()}")

    load_kwargs = {
        "trust_remote_code": True,
    }

    if torch.cuda.is_available():
        load_kwargs["device_map"] = {"": "cuda"}
        rospy.loginfo(f"[Init] GPU: {torch.cuda.get_device_name(0)}")
    else:
        load_kwargs["device_map"] = None
        rospy.logwarn("[Init] CUDA not available. Running on CPU.")

    if "4bit" not in model_name:
        load_kwargs["torch_dtype"] = torch.float16

    if revision:
        load_kwargs["revision"] = revision

    model = AutoModelForCausalLM.from_pretrained(
        model_name,
        **load_kwargs,
    )

    model.eval()


def warmup():
    rospy.loginfo("[Warmup] dummy inference...")

    input_size = int(config.get("input_size", 384))
    dummy = PILImage.new("RGB", (input_size, input_size), (128, 128, 128))

    with torch.inference_mode():
        encoded = model.encode_image(dummy)
        model.query(
            encoded,
            "What do you see?",
            settings={"max_tokens": 10},
        )

    rospy.loginfo("[Warmup] done")


def main():
    global config, mission_pub

    rospy.init_node("vlm_infer")

    config_path = os.path.join(
        os.path.dirname(__file__),
        "prompt.json",
    )

    with open(config_path, "r") as f:
        config = json.load(f)

    rospy.loginfo(f"[Config] config_path={config_path}")
    rospy.loginfo("[Config] candidate is forced to normal_drive")
    rospy.loginfo(f"[Config] infer_interval={config.get('infer_interval', 0.5)}")
    rospy.loginfo(f"[Config] input_size={config.get('input_size', 384)}")

    load_model()
    warmup()

    mission_pub = rospy.Publisher(
        "/vlm/mission",
        String,
        queue_size=1,
    )

    rospy.Subscriber(
        "/camera/color/image_raw/compressed",
        CompressedImage,
        image_callback,
        queue_size=1,
        buff_size=2**24,
    )

    rospy.on_shutdown(print_summary)

    rospy.loginfo("[Init] VLM latency node ready")
    rospy.loginfo("[Init] Subscribe: /camera/color/image_raw/compressed")
    rospy.loginfo("[Init] Publish: /vlm/mission")
    rospy.loginfo("[Init] Payload: candidate=normal_drive, vlm_raw, image_stamp, infer_time_ms")

    rospy.spin()


if __name__ == "__main__":
    main()