#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VLM Inference Node for LIMO Robot
- Subscribe: /camera/color/image_raw/compressed
- Publish: /vlm/mission (mission command for control)

Scene → Mission 매핑(추가 예정):
  clear → normal_drive
  cone → cone_avoidance
  obstacle → obstacle_stop
  crosswalk → crosswalk_stop
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import json
import os
import torch
from transformers import AutoModelForCausalLM, AutoTokenizer
from PIL import Image as PILImage
import time

# == 전역 변수 ==
model = None
tokenizer = None
config = None
mission_pub = None
last_infer_time = 0

def load_config(config_path: str) -> dict:
    with open(config_path, "r") as f:
        return json.load(f)


def load_model(model_id: str):
    rospy.loginfo(f"Loading model: {model_id}")
    tok = AutoTokenizer.from_pretrained(model_id, trust_remote_code=True)
    mdl = AutoModelForCausalLM.from_pretrained(
        model_id,
        trust_remote_code=True,
        torch_dtype=torch.float16,
    ).to("cuda" if torch.cuda.is_available() else "cpu")
    mdl.eval()
    rospy.loginfo("Model loaded successfully")
    return mdl, tok

def classify_scene(pil_img: PILImage.Image) -> str:
    global model, tokenizer, config

    with torch.no_grad():
        enc_image = model.encode_image(pil_img)
        prompt = config["prompt"]
        answer = model.answer_question(enc_image, prompt, tokenizer).strip().lower()
        return answer


def scene_to_mission(scene: str) -> str:
    global config
    mission_map = config.get("mission_map", {})

    # scene 키워드 매칭
    for keyword, mission in mission_map.items():
        if keyword in scene:
            return mission

    # 기본값 (Lane Detection)
    return mission_map.get("default", "normal_drive")


def image_callback(msg):
    global config, last_infer_time, mission_pub

    # 추론 주기 제어
    current_time = time.time()
    infer_interval = config.get("infer_interval", 0.5)
    if current_time - last_infer_time < infer_interval:
        return
    last_infer_time = current_time

    try:
        # CompressedImage -> OpenCV -> PIL
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv_img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(cv_img_rgb)

        # Scene 분류
        scene = classify_scene(pil_img)
        rospy.loginfo(f"[Scene] {scene}")

        # Mission 결정
        mission = scene_to_mission(scene)
        rospy.loginfo(f"[Mission] {mission}")

        # Publish
        mission_msg = String()
        mission_msg.data = mission
        mission_pub.publish(mission_msg)

    except Exception as e:
        rospy.logerr(f"Inference error: {e}")


def main():
    global model, tokenizer, config, mission_pub

    # Config 로드
    config_path = os.path.join(os.path.dirname(__file__), "prompt.json")
    config = load_config(config_path)

    # 모델 로드
    model, tokenizer = load_model(config["model"])
    device = "cuda" if torch.cuda.is_available() else "cpu"

    # ROS 초기화
    rospy.init_node('vlm_infer')

    # Publisher
    mission_pub = rospy.Publisher('/vlm/mission', String, queue_size=1)

    # Subscriber
    rospy.Subscriber(
        '/camera/color/image_raw/compressed',
        CompressedImage,
        image_callback,
        queue_size=1,
        buff_size=2 ** 24
    )

    rospy.loginfo(f"VLM Inference Node Started (device: {device})")
    rospy.loginfo(f"Inference interval: {config.get('infer_interval', 0.5)}s")
    rospy.spin()

if __name__ == '__main__':
    main()
