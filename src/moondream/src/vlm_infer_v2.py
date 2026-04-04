#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VLM Inference Node — LIMO AutoRace
─────────────────────────────────────────────────────────────────────────────
Sub : /camera/color/image_raw/compressed  (CompressedImage)
Pub : /vlm/mission                        (String, JSON payload)

Pipeline
  image_callback
      │
      ├─ [1] Preprocess  : decompress → resize(384×384)
      ├─ [2] VLM Infer   : open-ended prompt → raw_text + token confidence
      ├─ [3] Parser      : must/must_not rule → candidate mission
      ├─ [4] Conf Check  : min-token-prob < threshold → fallback=True
      ├─ [5] Temp Buffer : sliding-window majority vote → activated_mission
      └─ [6] Publish     : JSON{mission, confidence, fallback, timestamps}

JSON payload schema
  {
    "mission"            : str,    # one of VALID_MISSIONS
    "confidence"         : float,  # min softmax prob over generated tokens
    "fallback"           : bool,   # True → decision node uses rule-base only
    "vlm_raw"            : str,    # raw VLM answer (for debug)
    "image_stamp_sec"    : int,    # camera header.stamp  (seconds)
    "image_stamp_nsec"   : int,    # camera header.stamp  (nanoseconds)
    "vlm_publish_time"   : float   # rospy.Time.now() just before publish
  }

변경 이력 (v2)
  - time.time() → rospy.Time.now().to_sec()  (분산 환경 시간 동기화)
  - fallback / exception 시에도 activator.update("normal_drive") 호출
  - 파서 규칙 강화: anchor 키워드 도입, must_all 조건 추가
  - resize 378 → 384  (SigLIP 네이티브 해상도, 이중 보간 제거)
  - text_model.generate() 미지원 시 answer_question() fallback
"""

import os
import json
import time
from collections import deque, Counter

import rospy
import cv2
import numpy as np
import torch
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from PIL import Image as PILImage
from transformers import AutoModelForCausalLM, AutoTokenizer

# ══════════════════════════════════════════════════════════════════════════════
# Globals
# ══════════════════════════════════════════════════════════════════════════════
model        = None
tokenizer    = None
config       = None
mission_pub  = None
activator    = None
last_infer_t = 0.0

# text_model.generate()가 output_scores를 지원하는지 여부 (warmup에서 판별)
USE_GENERATE_SCORES = True

VALID_MISSIONS = frozenset({
    "normal_drive",
    "cone_avoidance",
    "crosswalk_stop",
    "sign_left",
    "sign_right",
})


# ══════════════════════════════════════════════════════════════════════════════
# [5] Temporal Confirmation Buffer
# ══════════════════════════════════════════════════════════════════════════════
class MissionActivator:
    """
    Sliding-window majority vote.
    미션이 window 프레임 중 threshold 이상 등장해야 활성화.
    → VLM이 멀리서 잠깐 감지한 미션은 buffer에서 묻힌다.
    """
    def __init__(self, window: int = 5, threshold: int = 3):
        self.buffer    = deque(maxlen=window)
        self.threshold = threshold

    def update(self, mission: str) -> str:
        self.buffer.append(mission)
        top_mission, count = Counter(self.buffer).most_common(1)[0]
        return top_mission if count >= self.threshold else "normal_drive"


# ══════════════════════════════════════════════════════════════════════════════
# [2] VLM Inference  +  Token Confidence
# ══════════════════════════════════════════════════════════════════════════════
def classify_scene_with_confidence(pil_img: PILImage.Image):
    """
    Open-ended 프롬프트로 VLM 추론.
    Returns
        raw_text   : str   — VLM 자유 응답
        confidence : float — 생성 토큰별 softmax 확률의 min값
                             (가장 불확실한 토큰 기준 → 보수적 신뢰도)
                             generate 미지원 시 1.0 (conf check 우회)
    """
    global model, tokenizer, config, USE_GENERATE_SCORES
    device = next(model.parameters()).device
    prompt = config["prompt"]

    # ── Path A: text_model.generate() + output_scores ─────────────────────
    if USE_GENERATE_SCORES:
        try:
            return _infer_with_scores(pil_img, prompt, device)
        except Exception as e:
            rospy.logwarn(
                f"[VLM] generate w/ scores failed ({e}), "
                "falling back to answer_question()"
            )
            USE_GENERATE_SCORES = False

    # ── Path B: answer_question() (confidence 미제공) ─────────────────────
    return _infer_answer_question(pil_img, prompt)


def _infer_with_scores(pil_img, prompt, device):
    with torch.inference_mode():
        enc_image = model.encode_image(pil_img)

        inputs = tokenizer(
            f"<image>\n\nQuestion: {prompt}\n\nAnswer:",
            return_tensors="pt",
        ).to(device)

        outputs = model.text_model.generate(
            **inputs,
            image_embeds=enc_image,
            max_new_tokens=config.get("max_new_tokens", 25),
            do_sample=False,
            output_scores=True,
            return_dict_in_generate=True,
        )

    input_len     = inputs["input_ids"].shape[1]
    generated_ids = outputs.sequences[0][input_len:]
    raw_text      = tokenizer.decode(generated_ids, skip_special_tokens=True).strip()

    token_probs = []
    for step, score in enumerate(outputs.scores):
        prob     = torch.softmax(score[0], dim=-1)
        token_id = generated_ids[step]
        token_probs.append(prob[token_id].item())

    confidence = min(token_probs) if token_probs else 0.0
    return raw_text, confidence


def _infer_answer_question(pil_img, prompt):
    """Moondream2 공식 API. confidence 정보 없으므로 1.0 반환."""
    with torch.inference_mode():
        raw_text = model.answer_question(
            model.encode_image(pil_img),
            prompt,
            tokenizer,
            max_new_tokens=config.get("max_new_tokens", 25),
        )
    return raw_text.strip(), 1.0


# ══════════════════════════════════════════════════════════════════════════════
# [3] Mission Parser  (must_any / must_all / must_not rule table)
# ══════════════════════════════════════════════════════════════════════════════
#
# 설계 원칙
#   - must_any : 하나라도 매칭되면 통과   (OR)
#   - must_all : 전부 매칭되어야 통과      (AND, 비어있으면 무조건 통과)
#   - must_not : 하나라도 매칭되면 차단
#   - priority : 낮을수록 우선
#
# 변경 사항 (v2)
#   - cone_avoidance: "cone"이 반드시 있어야 하고, "orange"는 보조
#   - crosswalk_stop: "crosswalk"·"zebra"·"pedestrian" 중 하나 필수,
#                     일반적인 "marking"·"stripe" 제거
#   - sign_left/right: "sign"·"arrow"·"turn" 중 하나가 반드시 동반
#
MISSION_RULES = [
    {
        "mission":  "cone_avoidance",
        "must_any": ["cone"],
        "must_all": [],
        "must_not": [],
        "priority": 1,
    },
    {
        "mission":  "crosswalk_stop",
        "must_any": ["crosswalk", "pedestrian", "zebra"],
        "must_all": [],
        "must_not": ["cone"],
        "priority": 2,
    },
    {
        "mission":  "sign_left",
        "must_any": ["sign", "arrow", "turn"],
        "must_all": ["left"],
        "must_not": ["cone", "crosswalk", "right"],
        "priority": 3,
    },
    {
        "mission":  "sign_right",
        "must_any": ["sign", "arrow", "turn"],
        "must_all": ["right"],
        "must_not": ["cone", "crosswalk", "left"],
        "priority": 3,
    },
]


def parse_mission_from_text(raw_text: str) -> str:
    text       = raw_text.lower()
    candidates = []

    for rule in MISSION_RULES:
        # must_any: 하나라도 있어야 함
        if not any(kw in text for kw in rule["must_any"]):
            continue
        # must_all: 전부 있어야 함
        if rule["must_all"] and not all(kw in text for kw in rule["must_all"]):
            continue
        # must_not: 하나라도 있으면 차단
        if any(kw in text for kw in rule["must_not"]):
            continue
        candidates.append(rule)

    if not candidates:
        return "normal_drive"

    return sorted(candidates, key=lambda r: r["priority"])[0]["mission"]


# ══════════════════════════════════════════════════════════════════════════════
# [4] Confidence Checker
# ══════════════════════════════════════════════════════════════════════════════
def check_confidence(confidence: float) -> bool:
    threshold = config.get("confidence_threshold", 0.55)
    if confidence < threshold:
        rospy.logwarn(
            f"[Conf] {confidence:.3f} < {threshold} → fallback"
        )
        return False
    return True


# ══════════════════════════════════════════════════════════════════════════════
# [6] Publish
# ══════════════════════════════════════════════════════════════════════════════
def publish_result(
    mission:      str,
    confidence:   float,
    fallback:     bool,
    image_stamp,               # rospy.Time
    vlm_raw:      str = "",
):
    payload = {
        "mission":          mission,
        "confidence":       round(confidence, 4),
        "fallback":         fallback,
        "vlm_raw":          vlm_raw,
        "image_stamp_sec":  image_stamp.secs,
        "image_stamp_nsec": image_stamp.nsecs,
        "vlm_publish_time": rospy.Time.now().to_sec(),   # ← ROS 시간 기준
    }
    msg      = String()
    msg.data = json.dumps(payload)
    mission_pub.publish(msg)

    rospy.loginfo(
        f"[Pub] mission={mission:16s} | conf={confidence:.3f} | fallback={fallback}"
    )


# ══════════════════════════════════════════════════════════════════════════════
# Image Callback
# ══════════════════════════════════════════════════════════════════════════════
def image_callback(msg: CompressedImage):
    global last_infer_t

    # ── 추론 주기 제어 ────────────────────────────────────────────────────
    now = time.time()
    if now - last_infer_t < config.get("infer_interval", 0.5):
        return
    last_infer_t = now

    image_stamp = msg.header.stamp
    t0          = time.time()

    try:
        # [1] Preprocess  (384×384 = SigLIP 네이티브 해상도)
        np_arr  = np.frombuffer(msg.data, np.uint8)
        cv_img  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        pil_img = PILImage.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))
        pil_img = pil_img.resize((384, 384), PILImage.LANCZOS)

        # [2] VLM Infer
        raw_text, confidence = classify_scene_with_confidence(pil_img)
        t1 = time.time()
        rospy.loginfo(
            f"[VLM] '{raw_text}' | infer={( t1 - t0 ) * 1000:.1f}ms | conf={confidence:.3f}"
        )

        # [3] Parse
        parsed_mission = parse_mission_from_text(raw_text)

        # [4] Confidence Check → fallback
        if not check_confidence(confidence):
            # ★ buffer에도 normal_drive를 기록하여 stale 방지
            activator.update("normal_drive")
            publish_result(
                mission="normal_drive", confidence=confidence,
                fallback=True, image_stamp=image_stamp, vlm_raw=raw_text,
            )
            return

        # [5] Temporal Buffer
        activated_mission = activator.update(parsed_mission)

        # [6] Publish
        publish_result(
            mission=activated_mission, confidence=confidence,
            fallback=False, image_stamp=image_stamp, vlm_raw=raw_text,
        )

    except Exception as e:
        rospy.logerr(f"[image_callback] {e}")
        # ★ exception 시에도 buffer 갱신
        activator.update("normal_drive")
        publish_result(
            mission="normal_drive", confidence=0.0,
            fallback=True, image_stamp=image_stamp,
        )


# ══════════════════════════════════════════════════════════════════════════════
# Warmup  (첫 추론 지연 제거 + generate 호환성 검증)
# ══════════════════════════════════════════════════════════════════════════════
def warmup():
    global USE_GENERATE_SCORES

    rospy.loginfo("[Warmup] Running dummy inference...")
    dummy = PILImage.new("RGB", (384, 384), color=(128, 128, 128))
    t0 = time.time()

    try:
        text, conf = _infer_with_scores(
            dummy,
            config["prompt"],
            next(model.parameters()).device,
        )
        rospy.loginfo(
            f"[Warmup] generate w/ scores OK "
            f"({(time.time()-t0)*1000:.0f}ms, conf={conf:.3f})"
        )
    except Exception as e:
        USE_GENERATE_SCORES = False
        rospy.logwarn(
            f"[Warmup] generate w/ scores FAILED ({e}), "
            "using answer_question() (confidence disabled)"
        )
        t0 = time.time()
        _infer_answer_question(dummy, config["prompt"])
        rospy.loginfo(f"[Warmup] answer_question() OK ({(time.time()-t0)*1000:.0f}ms)")


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════
def main():
    global model, tokenizer, config, mission_pub, activator

    # Config
    config_path = os.path.join(os.path.dirname(__file__), "prompt.json")
    with open(config_path) as f:
        config = json.load(f)

    # ROS
    rospy.init_node("vlm_infer")

    # Model
    rospy.loginfo(f"[Init] Loading {config['model']} ...")
    tokenizer = AutoTokenizer.from_pretrained(
        config["model"], trust_remote_code=True
    )
    model = AutoModelForCausalLM.from_pretrained(
        config["model"],
        trust_remote_code=True,
        torch_dtype=torch.float16,
    ).to("cuda" if torch.cuda.is_available() else "cpu")
    model.eval()
    rospy.loginfo("[Init] Model loaded")

    # Temporal buffer
    activator = MissionActivator(
        window    = config.get("buffer_window",    5),
        threshold = config.get("buffer_threshold", 3),
    )

    # Warmup  (generate 호환성도 여기서 판별)
    warmup()

    # Pub / Sub
    mission_pub = rospy.Publisher("/vlm/mission", String, queue_size=1)
    rospy.Subscriber(
        "/camera/color/image_raw/compressed",
        CompressedImage,
        image_callback,
        queue_size=1,
        buff_size=2 ** 24,
    )

    rospy.loginfo(
        f"[Init] VLM Node Ready | "
        f"interval={config.get('infer_interval',0.5)}s | "
        f"buffer={config.get('buffer_window',5)}/{config.get('buffer_threshold',3)} | "
        f"conf_th={config.get('confidence_threshold',0.55)} | "
        f"scores={'generate' if USE_GENERATE_SCORES else 'answer_question'}"
    )
    rospy.spin()


if __name__ == "__main__":
    main()
