#!/usr/bin/env python3.10
"""
Moondream2 - 이미지 파일 로컬 추론
Usage:
  ./infer_image.py
  ./infer_image.py --image /path/to/image.jpg
"""

import argparse
import json
import os

import torch
import cv2
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
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=str, help="이미지 파일 경로 (미지정 시 config.json 사용)")
    parser.add_argument(
        "--config",
        type=str,
        default=os.path.join(os.path.dirname(__file__), "config.json"),
    )
    args = parser.parse_args()

    config = load_config(args.config)
    model, tokenizer = load_model(config["model"])

    image_path = args.image or config["image_path"]
    image = Image.open(image_path).convert("RGB")
    print(f"이미지 로드: {image_path}")

    run_inference(model, tokenizer, image, config)
    


if __name__ == "__main__":
    main()
    
