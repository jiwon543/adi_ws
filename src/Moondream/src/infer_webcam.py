#!/usr/bin/env python3.10
"""
Moondream2 - 실시간 웹캠 로컬 추론
Space 키: 현재 프레임 캡처 후 추론
q 키: 종료
Usage:
  ./infer_webcam.py
"""

import json
import os
import sys

import cv2
import torch
from transformers import AutoModelForCausalLM, AutoTokenizer
from PIL import Image


def load_config(config_path: str) -> dict:
    with open(config_path, "r") as f:
        return json.load(f)


def load_model(model_id: str):
    print(f"모델 로딩 중: {model_id}")
    tokenizer = AutoTokenizer.from_pretrained(model_id, trust_remote_code=True)
    model = AutoModelForCausalLM.from_pretrained(
        model_id,
        trust_remote_code=True,
        torch_dtype=torch.float16,
    ).to("cuda" if torch.cuda.is_available() else "cpu")
    model.eval()
    return model, tokenizer


def run_inference(model, tokenizer, image: Image.Image, config: dict):
    enc_image = model.encode_image(image)
    prompts = config["prompts"]

    if prompts.get("caption", False):
        print("\n[Caption]")
        print(model.answer_question(enc_image, "Describe this image.", tokenizer))

    for question in prompts.get("questions", []):
        print(f"\n[Q] {question}")
        print(f"[A] {model.answer_question(enc_image, question, tokenizer)}")


def main():
    config_path = os.path.join(os.path.dirname(__file__), "config.json")
    config = load_config(config_path)
    model, tokenizer = load_model(config["model"])

    cam_index = config.get("webcam_index", 0)
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f"웹캠 {cam_index}번을 열 수 없습니다.")
        sys.exit(1)

    print("웹캠 시작. [Space] 캡처 및 추론 / [q] 종료")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        cv2.imshow("Webcam - Space: infer / q: quit", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        elif key == ord(" "):
            print("\n--- 추론 시작 ---")
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(rgb)
            run_inference(model, tokenizer, image, config)
            print("--- 추론 완료 ---")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
