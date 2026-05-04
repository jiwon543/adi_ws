#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VLM Inference Node v3 — LIMO AutoRace
─────────────────────────────────────────────────────────────────────────────
Sub : /camera/color/image_raw/compressed  (CompressedImage)
Pub : /vlm/mission                        (String, JSON payload)

Missions
  normal_drive   : 차선 추종 (기본값)
  cone_avoidance : 콘 사이 회피 주행
  crosswalk_stop : 노란 횡단보도 정지
  obstacle_stop  : 장난감 차 장애물 긴급 정지

Pipeline
  image_callback
      ├─ [1] Preprocess  : decompress → ROI crop → resize(384×384)
      ├─ [2] VLM Infer   : open-ended prompt → raw_text + confidence
      ├─ [3] Parser      : keyword rule table → candidate mission
      ├─ [4] Conf Gate   : confidence < threshold → fallback
      ├─ [5] Temp Buffer : sliding-window majority vote → activated
      └─ [6] Publish     : JSON payload

변경 이력 (v3)
  - confidence: answer_question() 후 scoring forward pass로 추정
    → generate(output_scores) 미지원 revision에서도 동작
  - 미션 정리: sign_left/right 제거, obstacle_stop 추가
  - 파서: obstacle_stop 규칙 추가 (car, vehicle, blocked 등)
  - latency 측정 코드 제거
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

VALID_MISSIONS = frozenset({
    "normal_drive",
    "cone_avoidance",
    "crosswalk_stop",
    "obstacle_stop",
})

# confidence 추출 방식 (warmup에서 결정)
#   "generate"  : text_model.generate(output_scores=True)
#   "scoring"   : answer_question() + scoring forward pass
#   "none"      : answer_question() only, conf=1.0
CONF_MODE = "generate"


# ══════════════════════════════════════════════════════════════════════════════
# [5] Temporal Confirmation Buffer
# ══════════════════════════════════════════════════════════════════════════════
class MissionActivator:
    def __init__(self, window: int = 5, threshold: int = 3):
        self.buffer    = deque(maxlen=window)
        self.threshold = threshold

    def update(self, mission: str) -> str:
        self.buffer.append(mission)
        top, count = Counter(self.buffer).most_common(1)[0]
        return top if count >= self.threshold else "normal_drive"


# ══════════════════════════════════════════════════════════════════════════════
# [2] VLM Inference + Confidence
# ══════════════════════════════════════════════════════════════════════════════
def classify_scene(pil_img: PILImage.Image):
    """
    Returns (raw_text, confidence).
    CONF_MODE에 따라 3가지 경로.
    """
    global CONF_MODE
    prompt = config["prompt"]
    device = next(model.parameters()).device

    if CONF_MODE == "generate":
        try:
            return _infer_generate(pil_img, prompt, device)
        except Exception as e:
            rospy.logwarn(f"[VLM] generate failed ({e}), trying scoring mode")
            CONF_MODE = "scoring"

    if CONF_MODE == "scoring":
        try:
            return _infer_scoring(pil_img, prompt, device)
        except Exception as e:
            rospy.logwarn(f"[VLM] scoring failed ({e}), falling back to none")
            CONF_MODE = "none"

    return _infer_plain(pil_img, prompt)


# ── Path A: generate + output_scores ──────────────────────────────────────
def _infer_generate(pil_img, prompt, device):
    with torch.inference_mode():
        enc = model.encode_image(pil_img)
        inputs = tokenizer(
            f"<image>\n\nQuestion: {prompt}\n\nAnswer:",
            return_tensors="pt",
        ).to(device)

        out = model.text_model.generate(
            **inputs,
            image_embeds=enc,
            max_new_tokens=config.get("max_new_tokens", 40),
            do_sample=False,
            output_scores=True,
            return_dict_in_generate=True,
        )

    input_len = inputs["input_ids"].shape[1]
    gen_ids   = out.sequences[0][input_len:]
    raw_text  = tokenizer.decode(gen_ids, skip_special_tokens=True).strip()

    probs = []
    for i, score in enumerate(out.scores):
        p = torch.softmax(score[0], dim=-1)
        probs.append(p[gen_ids[i]].item())

    return raw_text, min(probs) if probs else 0.0


# ── Path B: answer_question + scoring forward pass ────────────────────────
def _infer_scoring(pil_img, prompt, device):
    """
    1) answer_question()으로 텍스트 생성
    2) 전체 시퀀스를 forward pass → 토큰별 log-prob → min prob 계산
    """
    max_tok = config.get("max_new_tokens", 40)
    with torch.inference_mode():
        enc      = model.encode_image(pil_img)
        raw_text = model.answer_question(enc, prompt, tokenizer, max_new_tokens=max_tok)

    raw_text = raw_text.strip()

    # truncation 안전장치
    words = raw_text.split()
    if len(words) > max_tok * 2:
        raw_text = " ".join(words[: max_tok * 2])

    # scoring pass: 생성된 텍스트를 포함한 전체 시퀀스의 log-prob
    try:
        confidence = _score_sequence(prompt, raw_text, device)
    except Exception:
        confidence = 1.0   # scoring 실패 시 gate 우회

    return raw_text, confidence


def _score_sequence(prompt, answer, device):
    """
    프롬프트+답변 전체를 tokenize → forward → 답변 토큰의 min prob.
    image embedding 없이 텍스트만으로 scoring하므로 정확한 conditional prob는 아니지만,
    모델이 확신 없는 토큰을 잡아내기엔 충분.
    """
    full_text   = f"<image>\n\nQuestion: {prompt}\n\nAnswer: {answer}"
    prompt_text = f"<image>\n\nQuestion: {prompt}\n\nAnswer:"

    full_ids   = tokenizer(full_text,   return_tensors="pt").input_ids.to(device)
    prompt_ids = tokenizer(prompt_text,  return_tensors="pt").input_ids.to(device)
    ans_start  = prompt_ids.shape[1]

    if full_ids.shape[1] <= ans_start:
        return 1.0

    with torch.inference_mode():
        logits = model.text_model(full_ids).logits   # (1, seq_len, vocab)

    # 각 답변 토큰 위치에서 해당 토큰의 softmax prob
    probs = []
    for i in range(ans_start, full_ids.shape[1]):
        p = torch.softmax(logits[0, i - 1], dim=-1)
        probs.append(p[full_ids[0, i]].item())

    return min(probs) if probs else 1.0


# ── Path C: plain answer_question (no confidence) ─────────────────────────
def _infer_plain(pil_img, prompt):
    max_tok = config.get("max_new_tokens", 40)
    with torch.inference_mode():
        raw = model.answer_question(
            model.encode_image(pil_img), prompt, tokenizer,
            max_new_tokens=max_tok,
        )
    raw = raw.strip()
    words = raw.split()
    if len(words) > max_tok * 2:
        raw = " ".join(words[: max_tok * 2])
    return raw, 1.0


# ══════════════════════════════════════════════════════════════════════════════
# [3] Mission Parser
# ══════════════════════════════════════════════════════════════════════════════
MISSION_RULES = [
    {
        "mission":  "obstacle_stop",
        "must_any": ["car", "vehicle", "toy", "blocked", "obstruction", "automobile"],
        "must_all": [],
        "must_not": ["cone"],
        "priority": 3,
    },
    {
        "mission":  "crosswalk_stop",
        "must_any": ["crosswalk", "zebra", "yellow","yellow line", "yellow stripe"],
        "must_all": [],
        "must_not": ["cone"],
        "priority": 1,
    },
    {
        "mission":  "cone_avoidance",
        "must_any": ["cone", "traffic cone", "orange cone"],
        "must_all": [],
        "must_not": [],
        "priority": 2,
    },
]


def parse_mission(raw_text: str) -> str:
    text = raw_text.lower()
    hits = []
    for rule in MISSION_RULES:
        if not any(kw in text for kw in rule["must_any"]):
            continue
        if rule["must_all"] and not all(kw in text for kw in rule["must_all"]):
            continue
        if any(kw in text for kw in rule["must_not"]):
            continue
        hits.append(rule)

    if not hits:
        return "normal_drive"
    return sorted(hits, key=lambda r: r["priority"])[0]["mission"]


# ══════════════════════════════════════════════════════════════════════════════
# [4] Confidence Gate
# ══════════════════════════════════════════════════════════════════════════════
def confidence_ok(conf: float) -> bool:
    th = config.get("confidence_threshold", 0.45)
    if conf < th:
        rospy.logwarn(f"[Conf] {conf:.3f} < {th} → fallback")
        return False
    return True


# ══════════════════════════════════════════════════════════════════════════════
# [6] Publish
# ══════════════════════════════════════════════════════════════════════════════
def publish_result(mission, confidence, fallback, image_stamp, vlm_raw=""):
    payload = {
        "mission":          mission,
        "confidence":       round(confidence, 4),
        "fallback":         fallback,
        "vlm_raw":          vlm_raw,
        "image_stamp_sec":  image_stamp.secs,
        "image_stamp_nsec": image_stamp.nsecs,
        "vlm_publish_time": rospy.Time.now().to_sec(),
    }
    msg      = String()
    msg.data = json.dumps(payload)
    mission_pub.publish(msg)
    rospy.loginfo(f"[Pub] {mission:16s} | conf={confidence:.3f} | fb={fallback}")


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

    try:
        # [1] Preprocess
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_img is None:
            rospy.logwarn("[Callback] imdecode failed — skipping frame")
            return

        h, w    = cv_img.shape[:2]
        roi_top = config.get("roi_top_ratio", 0.6)
        cv_img  = cv_img[int(h * roi_top):, :]

        pil_img = PILImage.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))
        pil_img = pil_img.resize((384, 384), PILImage.LANCZOS)

        # [2] Infer
        raw_text, confidence = classify_scene(pil_img)

        # [3] Parse
        parsed = parse_mission(raw_text)

        # [4] Confidence gate
        if not confidence_ok(confidence):
            activator.update("normal_drive")
            publish_result("normal_drive", confidence, True, image_stamp, raw_text)
            return

        # [5] Temporal buffer
        activated = activator.update(parsed)

        # [6] Publish
        publish_result(activated, confidence, False, image_stamp, raw_text)

    except Exception as e:
        rospy.logerr(f"[Callback] {e}")
        activator.update("normal_drive")
        publish_result("normal_drive", 0.0, True, image_stamp)


# ══════════════════════════════════════════════════════════════════════════════
# Warmup — confidence 모드 자동 판별
# ══════════════════════════════════════════════════════════════════════════════
def warmup():
    global CONF_MODE
    dummy  = PILImage.new("RGB", (384, 384), color=(128, 128, 128))
    device = next(model.parameters()).device
    prompt = config["prompt"]

    # 1) generate + scores
    try:
        _, c = _infer_generate(dummy, prompt, device)
        CONF_MODE = "generate"
        rospy.loginfo(f"[Warmup] generate+scores OK (conf={c:.3f})")
        return
    except Exception as e:
        rospy.logwarn(f"[Warmup] generate failed: {e}")

    # 2) scoring forward pass
    try:
        _, c = _infer_scoring(dummy, prompt, device)
        CONF_MODE = "scoring"
        rospy.loginfo(f"[Warmup] scoring pass OK (conf={c:.3f})")
        return
    except Exception as e:
        rospy.logwarn(f"[Warmup] scoring failed: {e}")

    # 3) plain
    CONF_MODE = "none"
    _infer_plain(dummy, prompt)
    rospy.logwarn("[Warmup] confidence disabled, using answer_question() only")


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════
def main():
    global model, tokenizer, config, mission_pub, activator

    config_path = os.path.join(os.path.dirname(__file__), "prompt_v3.json")
    with open(config_path) as f:
        config = json.load(f)

    rospy.init_node("vlm_infer")

    rospy.loginfo(f"[Init] Loading {config['model']} ...")
    tokenizer = AutoTokenizer.from_pretrained(
        config["model"], trust_remote_code=True,
    )
    model = AutoModelForCausalLM.from_pretrained(
        config["model"],
        trust_remote_code=True,
        torch_dtype=torch.float16,
    ).to("cuda" if torch.cuda.is_available() else "cpu")
    model.eval()
    rospy.loginfo("[Init] Model loaded")

    activator = MissionActivator(
        window    = config.get("buffer_window", 5),
        threshold = config.get("buffer_threshold", 3),
    )

    warmup()

    mission_pub = rospy.Publisher("/vlm/mission", String, queue_size=1)
    rospy.Subscriber(
        "/camera/color/image_raw/compressed",
        CompressedImage,
        image_callback,
        queue_size=1,
        buff_size=2 ** 24,
    )

    rospy.loginfo(
        f"[Init] VLM v3 Ready | "
        f"conf_mode={CONF_MODE} | "
        f"interval={config.get('infer_interval',0.5)}s | "
        f"buffer={config.get('buffer_window',5)}/{config.get('buffer_threshold',3)} | "
        f"conf_th={config.get('confidence_threshold',0.45)}"
    )
    rospy.spin()


if __name__ == "__main__":
    main()