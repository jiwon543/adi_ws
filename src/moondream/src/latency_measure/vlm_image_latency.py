#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
vlm_image_latency.py

Purpose:
  - 로봇 없이 로컬 이미지 set으로 Moondream2 VLM inference time 측정
  - simple / complex 이미지 그룹 비교
  - simple / complex 그룹별 max_tokens 따로 설정 가능
  - 기존 ROS latency_vlm.py 측정 방식에 최대한 맞춤
    * no torch.cuda.synchronize() by default
    * cv2.imread -> cv2.resize -> BGR2RGB -> PIL
    * encode_image + query 구간만 측정
  - raw 결과 CSV + image별 summary CSV + group별 summary CSV 저장

Folder structure:
  test_images/
  ├─ simple/
  │  ├─ simple_01.jpg
  │  └─ ...
  └─ complex/
     ├─ complex_01.jpg
     └─ ...

Example:
  # simple/complex 둘 다 25 tokens
  python vlm_image_latency.py --root ./test_images --repeat 10 --max-tokens 25

  # simple은 10 tokens, complex는 25 tokens
  python vlm_image_latency.py --root ./test_images --repeat 10 --simple-max-tokens 10 --complex-max-tokens 25

  # strict GPU timing
  python vlm_image_latency.py --root ./test_images --repeat 10 --sync-cuda
"""

import os
import csv
import time
import glob
import argparse
import statistics
from datetime import datetime

import cv2
import torch
from PIL import Image as PILImage
from transformers import AutoModelForCausalLM


IMAGE_EXTS = ["*.jpg", "*.jpeg", "*.png", "*.bmp", "*.webp"]


def mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def std(xs):
    if len(xs) < 2:
        return 0.0
    return statistics.stdev(xs)


def get_image_paths(folder):
    paths = []
    for ext in IMAGE_EXTS:
        paths.extend(glob.glob(os.path.join(folder, ext)))
    return sorted(paths)


def maybe_cuda_sync(use_sync):
    if use_sync and torch.cuda.is_available():
        torch.cuda.synchronize()


def load_model(model_name, revision):
    print(f"[Init] Loading model: {model_name}")
    print(f"[Init] Revision     : {revision}")
    print(f"[Init] CUDA available: {torch.cuda.is_available()}")

    load_kwargs = {
        "trust_remote_code": True,
    }

    if revision:
        load_kwargs["revision"] = revision

    if torch.cuda.is_available():
        print(f"[Init] GPU: {torch.cuda.get_device_name(0)}")
        load_kwargs["device_map"] = {"": "cuda"}

        if "4bit" not in model_name.lower():
            load_kwargs["torch_dtype"] = torch.float16
            print("[Init] Precision    : FP16")
        else:
            print("[Init] Precision    : 4bit model setting")
    else:
        print("[Warn] CUDA not available. Running on CPU.")
        load_kwargs["device_map"] = None

    model = AutoModelForCausalLM.from_pretrained(
        model_name,
        **load_kwargs,
    )

    model.eval()

    try:
        first_param = next(model.parameters())
        print(f"[Init] Model dtype   : {first_param.dtype}")
        print(f"[Init] Model device  : {first_param.device}")
    except Exception:
        pass

    return model


def load_image_like_ros(image_path, input_size):
    """
    기존 ROS 코드 전처리 흐름에 맞춤:
      cv2.imread -> cv2.resize -> BGR2RGB -> PIL
    """
    cv_img = cv2.imread(image_path, cv2.IMREAD_COLOR)

    if cv_img is None:
        raise RuntimeError(f"cv2.imread failed: {image_path}")

    if input_size > 0:
        cv_img = cv2.resize(cv_img, (input_size, input_size))

    pil_img = PILImage.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))
    return pil_img


def run_one_inference(model, image, prompt, max_tokens):
    """
    측정 범위:
      model.encode_image(pil_img)
      model.query(encoded, prompt, settings={"max_tokens": max_tokens})
    """
    with torch.inference_mode():
        encoded = model.encode_image(image)

        result = model.query(
            encoded,
            prompt,
            settings={"max_tokens": max_tokens},
        )

    answer = result["answer"].strip()
    return answer


def warmup(model, input_size, prompt, max_tokens, warmup_count, use_sync):
    print(f"[Warmup] Running {warmup_count} dummy inferences...")
    print(f"[Warmup] max_tokens = {max_tokens}")

    dummy = PILImage.new("RGB", (input_size, input_size), (128, 128, 128))

    for i in range(warmup_count):
        maybe_cuda_sync(use_sync)
        _ = run_one_inference(model, dummy, prompt, max_tokens)
        maybe_cuda_sync(use_sync)

        print(f"[Warmup] {i + 1}/{warmup_count} done")

    print("[Warmup] Done")


def measure_image(
    model,
    group_name,
    image_path,
    prompt,
    max_tokens,
    input_size,
    repeat,
    use_sync,
    model_name,
    revision,
    timing_mode,
):
    image = load_image_like_ros(image_path, input_size)
    rows = []

    for rep_idx in range(1, repeat + 1):
        maybe_cuda_sync(use_sync)
        t0 = time.perf_counter()

        answer = run_one_inference(
            model=model,
            image=image,
            prompt=prompt,
            max_tokens=max_tokens,
        )

        maybe_cuda_sync(use_sync)
        t1 = time.perf_counter()

        infer_ms = (t1 - t0) * 1000.0

        row = {
            "group": group_name,
            "image_name": os.path.basename(image_path),
            "image_path": image_path,
            "repeat_idx": rep_idx,
            "infer_time_ms": infer_ms,
            "answer": answer,
            "model": model_name,
            "revision": revision,
            "prompt": prompt,
            "max_tokens": max_tokens,
            "input_size": input_size,
            "timing_mode": timing_mode,
        }
        rows.append(row)

        print(
            f"[{group_name:7s}] "
            f"{os.path.basename(image_path):25s} "
            f"repeat={rep_idx:02d}/{repeat} | "
            f"tokens={max_tokens:02d} | "
            f"{infer_ms:8.3f} ms | "
            f"answer={answer}"
        )

    return rows


def summarize_by_image(raw_rows):
    summary_rows = []

    key_to_times = {}
    key_to_tokens = {}

    for row in raw_rows:
        key = (row["group"], row["image_name"], row["image_path"])
        key_to_times.setdefault(key, []).append(row["infer_time_ms"])
        key_to_tokens[key] = row["max_tokens"]

    for (group, image_name, image_path), times in key_to_times.items():
        key = (group, image_name, image_path)

        summary_rows.append({
            "group": group,
            "image_name": image_name,
            "image_path": image_path,
            "max_tokens": key_to_tokens[key],
            "n": len(times),
            "mean_ms": mean(times),
            "std_ms": std(times),
            "min_ms": min(times),
            "max_ms": max(times),
        })

    return summary_rows


def summarize_by_group(raw_rows):
    summary_rows = []

    group_to_times = {}
    group_to_tokens = {}

    for row in raw_rows:
        group = row["group"]
        group_to_times.setdefault(group, []).append(row["infer_time_ms"])
        group_to_tokens[group] = row["max_tokens"]

    for group, times in group_to_times.items():
        summary_rows.append({
            "group": group,
            "max_tokens": group_to_tokens[group],
            "n": len(times),
            "mean_ms": mean(times),
            "std_ms": std(times),
            "min_ms": min(times),
            "max_ms": max(times),
        })

    return summary_rows


def save_csv(path, rows, fieldnames):
    os.makedirs(os.path.dirname(path), exist_ok=True)

    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for row in rows:
            out = {}
            for field in fieldnames:
                value = row.get(field, "")

                if isinstance(value, float):
                    out[field] = f"{value:.6f}"
                else:
                    out[field] = value

            writer.writerow(out)

    print(f"[Save] {path}")


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--root",
        type=str,
        default="./test_images",
        help="Root folder containing simple/ and complex/ folders",
    )

    parser.add_argument(
        "--output-dir",
        type=str,
        default="./vlm_latency_results",
        help="CSV output directory",
    )

    parser.add_argument(
        "--model",
        type=str,
        default="vikhyatk/moondream2",
        help="Hugging Face model name",
    )

    parser.add_argument(
        "--revision",
        type=str,
        default="2025-01-09",
        help="Model revision",
    )

    parser.add_argument(
        "--prompt",
        type=str,
        default=(
            "You are a car driver looking forward through the windshield.\n"
            "Describe the hazard you see on the road ahead in one short sentence."
        ),
        help="Prompt for VLM",
    )

    parser.add_argument(
        "--max-tokens",
        type=int,
        default=None,
        help=(
            "Common max output tokens. "
            "If set, overrides --simple-max-tokens and --complex-max-tokens."
        ),
    )

    parser.add_argument(
        "--simple-max-tokens",
        type=int,
        default=30,
        help="Max output tokens for simple images",
    )

    parser.add_argument(
        "--complex-max-tokens",
        type=int,
        default=30,
        help="Max output tokens for complex images",
    )

    parser.add_argument(
        "--input-size",
        type=int,
        default=384,
        help="Resize image to input_size x input_size. Use 0 to disable resize.",
    )

    parser.add_argument(
        "--repeat",
        type=int,
        default=10,
        help="Number of repeated inferences per image",
    )

    parser.add_argument(
        "--warmup",
        type=int,
        default=3,
        help="Number of warm-up inferences",
    )

    parser.add_argument(
        "--sync-cuda",
        action="store_true",
        help=(
            "Use torch.cuda.synchronize() before and after inference. "
            "Default is False to reproduce ROS-style timing."
        ),
    )

    args = parser.parse_args()

    # --max-tokens를 주면 simple/complex 둘 다 같은 값으로 override
    if args.max_tokens is not None:
        args.simple_max_tokens = args.max_tokens
        args.complex_max_tokens = args.max_tokens

    simple_dir = os.path.join(args.root, "simple")
    complex_dir = os.path.join(args.root, "complex")

    simple_images = get_image_paths(simple_dir)
    complex_images = get_image_paths(complex_dir)

    timing_mode = "cuda_sync" if args.sync_cuda else "legacy_no_cuda_sync"

    print("=" * 70)
    print("[Config]")
    print(f"  root              : {args.root}")
    print(f"  simple_dir        : {simple_dir}")
    print(f"  complex_dir       : {complex_dir}")
    print(f"  simple n          : {len(simple_images)}")
    print(f"  complex n         : {len(complex_images)}")
    print(f"  repeat            : {args.repeat}")
    print(f"  total infer       : {(len(simple_images) + len(complex_images)) * args.repeat}")
    print(f"  model             : {args.model}")
    print(f"  revision          : {args.revision}")
    print(f"  prompt            : {args.prompt}")
    print(f"  common max_tokens : {args.max_tokens}")
    print(f"  simple_max_tokens : {args.simple_max_tokens}")
    print(f"  complex_max_tokens: {args.complex_max_tokens}")
    print(f"  input_size        : {args.input_size}")
    print(f"  timing            : {timing_mode}")
    print("=" * 70)

    if len(simple_images) == 0:
        raise RuntimeError(f"No images found in {simple_dir}")

    if len(complex_images) == 0:
        raise RuntimeError(f"No images found in {complex_dir}")

    model = load_model(args.model, args.revision)

    # warm-up은 둘 중 더 큰 토큰 수로 수행
    warmup_max_tokens = max(args.simple_max_tokens, args.complex_max_tokens)

    warmup(
        model=model,
        input_size=args.input_size if args.input_size > 0 else 384,
        prompt=args.prompt,
        max_tokens=warmup_max_tokens,
        warmup_count=args.warmup,
        use_sync=args.sync_cuda,
    )

    raw_rows = []

    for image_path in simple_images:
        raw_rows.extend(
            measure_image(
                model=model,
                group_name="simple",
                image_path=image_path,
                prompt=args.prompt,
                max_tokens=args.simple_max_tokens,
                input_size=args.input_size,
                repeat=args.repeat,
                use_sync=args.sync_cuda,
                model_name=args.model,
                revision=args.revision,
                timing_mode=timing_mode,
            )
        )

    for image_path in complex_images:
        raw_rows.extend(
            measure_image(
                model=model,
                group_name="complex",
                image_path=image_path,
                prompt=args.prompt,
                max_tokens=args.complex_max_tokens,
                input_size=args.input_size,
                repeat=args.repeat,
                use_sync=args.sync_cuda,
                model_name=args.model,
                revision=args.revision,
                timing_mode=timing_mode,
            )
        )

    image_summary_rows = summarize_by_image(raw_rows)
    group_summary_rows = summarize_by_group(raw_rows)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    raw_csv = os.path.join(args.output_dir, f"vlm_latency_raw_{timestamp}.csv")
    image_summary_csv = os.path.join(args.output_dir, f"vlm_latency_image_summary_{timestamp}.csv")
    group_summary_csv = os.path.join(args.output_dir, f"vlm_latency_group_summary_{timestamp}.csv")

    save_csv(
        raw_csv,
        raw_rows,
        fieldnames=[
            "group",
            "image_name",
            "image_path",
            "repeat_idx",
            "infer_time_ms",
            "answer",
            "model",
            "revision",
            "prompt",
            "max_tokens",
            "input_size",
            "timing_mode",
        ],
    )

    save_csv(
        image_summary_csv,
        image_summary_rows,
        fieldnames=[
            "group",
            "image_name",
            "image_path",
            "max_tokens",
            "n",
            "mean_ms",
            "std_ms",
            "min_ms",
            "max_ms",
        ],
    )

    save_csv(
        group_summary_csv,
        group_summary_rows,
        fieldnames=[
            "group",
            "max_tokens",
            "n",
            "mean_ms",
            "std_ms",
            "min_ms",
            "max_ms",
        ],
    )

    print("\n" + "=" * 70)
    print("[Group Summary]")
    for row in group_summary_rows:
        print(
            f"{row['group']:7s} | "
            f"tokens={row['max_tokens']:3d} | "
            f"n={row['n']:3d} | "
            f"mean={row['mean_ms']:8.3f} ms | "
            f"std={row['std_ms']:8.3f} ms | "
            f"min={row['min_ms']:8.3f} ms | "
            f"max={row['max_ms']:8.3f} ms"
        )
    print("=" * 70)

    print("\n[Done]")
    print(f"Raw CSV          : {raw_csv}")
    print(f"Image summary CSV: {image_summary_csv}")
    print(f"Group summary CSV: {group_summary_csv}")


if __name__ == "__main__":
    main()