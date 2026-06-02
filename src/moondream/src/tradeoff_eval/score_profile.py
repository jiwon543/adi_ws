#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
score_profile.py — 기존 latency_profile.csv 한 개를 읽어 trade-off 곡선의 '한 점' 산출.

latency_profile.csv (vlm_latency_profiler.py 출력) 컬럼 사용:
  class_name(=GT), image, repeat_idx, encode_ms, decode_ms, total_inference_ms,
  num_tokens, answer, prompt, max_tokens

- 정확도: greedy라 answer가 repeat 간 동일 → 이미지당 1개(첫 등장)로 (GT, pred) 구성.
- 속도   : 모든 repeat 행의 total_inference_ms 로 p95/mean/max.

사용:
  python3 score_profile.py <latency_profile.csv> [more.csv ...]
  인자 없으면 PyCharm 프로젝트의 기존 run 들을 자동 탐색.
"""

import csv
import os
import sys
import glob

import metrics


# 기본 탐색 경로 (인자 없을 때)
DEFAULT_GLOB = (
    "/mnt/c/Users/jiwlee/PycharmProjects/Moondream/"
    "outputs/latency_profile/run_*/latency_profile.csv"
)


def load_rows(path):
    with open(path, encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def score_one(path):
    rows = load_rows(path)
    if not rows:
        return None

    prompt = rows[0].get("prompt", "")
    cap = rows[0].get("max_tokens", "")

    # ── 정확도: 이미지당 첫 답변 ──
    seen = {}
    for r in rows:
        key = (r["class_name"], r["image"])
        if key not in seen:
            seen[key] = r["answer"]
    items = [(gt, metrics.predict_class(ans)) for (gt, _img), ans in seen.items()]

    # ── 속도: 전 repeat ──
    lat = [float(r["total_inference_ms"]) for r in rows if r.get("total_inference_ms")]
    enc = [float(r["encode_ms"]) for r in rows if r.get("encode_ms")]
    dec = [float(r["decode_ms"]) for r in rows if r.get("decode_ms")]
    toks = [int(r["num_tokens"]) for r in rows if r.get("num_tokens")]

    return {
        "path": path,
        "prompt": prompt,
        "cap": cap,
        "n_images": len(seen),
        "acc": metrics.accuracy(items),
        "acc_bal": metrics.balanced_accuracy(items),
        "acc_w": metrics.weighted_accuracy(items),
        "recall": metrics.recall_per_class(items),
        "confusion": metrics.confusion(items),
        "lat": metrics.latency_summary(lat),
        "enc_mean": sum(enc) / len(enc) if enc else 0.0,
        "dec_mean": sum(dec) / len(dec) if dec else 0.0,
        "tok_mean": sum(toks) / len(toks) if toks else 0.0,
    }


def print_point(s):
    print("=" * 68)
    print(f"파일   : {os.path.basename(os.path.dirname(s['path']))}/{os.path.basename(s['path'])}")
    print(f"세팅   : cap={s['cap']} | prompt=\"{s['prompt'][:54]}\"")
    print(f"이미지 : {s['n_images']}장 | 평균 토큰={s['tok_mean']:.1f}")
    print("-" * 68)
    print("[정확도 축]")
    print(f"  Acc(전체)   = {s['acc']*100:5.1f}%")
    print(f"  Acc_bal ★   = {s['acc_bal']*100:5.1f}%   (클래스 recall 평균)")
    print(f"  Acc_w(안전) = {s['acc_w']*100:5.1f}%")
    print(f"  클래스별 recall:")
    for c in metrics.CLASSES:
        r = s["recall"][c]
        print(f"    {c:9s}: {'   -' if r is None else f'{r*100:5.1f}%'}")
    print(f"  혼동행렬 (행=GT, 열=pred):")
    header = "          " + "".join(f"{c:>9s}" for c in metrics.CLASSES)
    print(header)
    for g in metrics.CLASSES:
        cells = "".join(f"{s['confusion'][g][p]:9d}" for p in metrics.CLASSES)
        print(f"    {g:7s}{cells}")
    print("-" * 68)
    print("[속도 축]")
    L = s["lat"]
    print(f"  L_p95 ★ = {L['p95']:7.1f} ms   (mean={L['mean']:.1f}, max={L['max']:.1f}, n={L['n']})")
    print(f"  분해    : encode={s['enc_mean']:.1f}ms (상수) + decode={s['dec_mean']:.1f}ms (∝토큰)")
    print("=" * 68)
    print(f"→ 곡선 점: ( L_p95={L['p95']:.0f}ms , Acc_bal={s['acc_bal']*100:.1f}% )\n")


def main():
    paths = sys.argv[1:] or sorted(glob.glob(DEFAULT_GLOB))
    if not paths:
        print("latency_profile.csv 를 찾지 못했습니다. 경로를 인자로 주세요.")
        return

    points = []
    for p in paths:
        s = score_one(p)
        if s:
            print_point(s)
            points.append(s)

    if len(points) > 1:
        print("\n##### 곡선 점 요약 (x=L_p95, y=Acc_bal) #####")
        print(f"{'cap':>5} {'L_p95(ms)':>10} {'Acc_bal':>8} {'Acc':>6}  파일")
        for s in sorted(points, key=lambda z: z["lat"]["p95"]):
            print(f"{str(s['cap']):>5} {s['lat']['p95']:>10.1f} "
                  f"{s['acc_bal']*100:>7.1f}% {s['acc']*100:>5.1f}%  "
                  f"{os.path.basename(os.path.dirname(s['path']))}")


if __name__ == "__main__":
    main()
