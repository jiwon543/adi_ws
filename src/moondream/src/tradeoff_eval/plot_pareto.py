#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_pareto.py — trade-off 곡선 (x=L_p95 ms, y=Acc_bal %) 작성.

입력: latency_profile.csv 또는 sweep_tokens.csv (score_profile.score_one 이 읽는 포맷)
  여러 개 주면 각각이 곡선의 한 점.

출력:
  1) results/curve_points.csv  — (cap, L_p95, acc_bal, acc, prompt)
  2) ASCII 산점도 (의존성 없이 항상)
  3) results/pareto.png         — matplotlib 있으면

사용:
  python3 plot_pareto.py                # 기본 run 자동 탐색
  python3 plot_pareto.py a.csv b.csv ...
  python3 plot_pareto.py --tbudget 800  # T_budget(ms) 세로선
"""

import csv
import os
import sys
import glob

import metrics
from score_profile import score_one, DEFAULT_GLOB

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "results")


def collect_points(paths):
    pts = []
    for p in paths:
        s = score_one(p)
        if s:
            pts.append(s)
    return sorted(pts, key=lambda z: z["lat"]["p95"])


def save_points_csv(points):
    os.makedirs(RESULTS_DIR, exist_ok=True)
    out = os.path.join(RESULTS_DIR, "curve_points.csv")
    with open(out, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["cap", "L_p95_ms", "L_mean_ms", "acc_bal", "acc", "acc_w",
                    "n_images", "tok_mean", "prompt"])
        for s in points:
            w.writerow([s["cap"], f"{s['lat']['p95']:.1f}", f"{s['lat']['mean']:.1f}",
                        f"{s['acc_bal']:.4f}", f"{s['acc']:.4f}", f"{s['acc_w']:.4f}",
                        s["n_images"], f"{s['tok_mean']:.1f}", s["prompt"]])
    return out


def ascii_scatter(points, tbudget=None, width=58, height=16):
    if not points:
        print("(점 없음)")
        return
    xs = [s["lat"]["p95"] for s in points]
    ys = [s["acc_bal"] * 100 for s in points]
    xmin, xmax = min(xs), max(xs)
    if tbudget is not None:
        xmin, xmax = min(xmin, tbudget), max(xmax, tbudget)
    ymin, ymax = min(ys), 100.0
    if xmax - xmin < 1e-9:
        xmax = xmin + 1
    if ymax - ymin < 1e-9:
        ymin = ymax - 1

    grid = [[" "] * width for _ in range(height)]

    def col(x):
        return min(width - 1, max(0, int((x - xmin) / (xmax - xmin) * (width - 1))))

    def row(y):
        return min(height - 1, max(0, int((ymax - y) / (ymax - ymin) * (height - 1))))

    if tbudget is not None:
        c = col(tbudget)
        for r in range(height):
            grid[r][c] = "|"
    for s in points:
        r = row(s["acc_bal"] * 100)
        c = col(s["lat"]["p95"])
        grid[r][c] = "*"

    print(f"\nTrade-off 곡선  (x=L_p95 ms →느림, y=Acc_bal %)")
    print(f"  y:[{ymin:.1f} .. {ymax:.1f}]%   x:[{xmin:.0f} .. {xmax:.0f}]ms"
          + (f"   T_budget={tbudget:.0f}ms(|)" if tbudget else ""))
    for r in range(height):
        ylab = ymax - (ymax - ymin) * r / (height - 1)
        print(f"{ylab:5.1f} |" + "".join(grid[r]))
    print("      +" + "-" * width)
    print("       " + f"{xmin:.0f}".ljust(width - 5) + f"{xmax:.0f}")


def main():
    args = [a for a in sys.argv[1:]]
    tbudget = None
    if "--tbudget" in args:
        i = args.index("--tbudget")
        tbudget = float(args[i + 1])
        del args[i:i + 2]

    paths = args or sorted(glob.glob(DEFAULT_GLOB))
    points = collect_points(paths)
    if not points:
        print("점을 만들 CSV가 없습니다.")
        return

    out = save_points_csv(points)
    print(f"[저장] {out}")
    print(f"\n{'cap':>5} {'L_p95(ms)':>10} {'Acc_bal':>8} {'Acc':>7}")
    for s in points:
        print(f"{str(s['cap']):>5} {s['lat']['p95']:>10.1f} "
              f"{s['acc_bal']*100:>7.1f}% {s['acc']*100:>6.1f}%")

    ascii_scatter(points, tbudget=tbudget)

    # matplotlib 있으면 png
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        xs = [s["lat"]["p95"] for s in points]
        ys = [s["acc_bal"] * 100 for s in points]
        caps = [s["cap"] for s in points]
        plt.figure(figsize=(7, 5))
        plt.plot(xs, ys, "-o", color="#6366f1")
        for x, y, c in zip(xs, ys, caps):
            plt.annotate(f"cap={c}", (x, y), fontsize=8,
                         textcoords="offset points", xytext=(5, 5))
        if tbudget is not None:
            plt.axvline(tbudget, color="red", ls="--", label=f"T_budget={tbudget:.0f}ms")
            plt.legend()
        plt.xlabel("L_p95 [ms]  (←빠름)")
        plt.ylabel("Acc_bal [%]")
        plt.title("VLM 속도-정확도 Trade-off")
        plt.grid(alpha=0.3)
        png = os.path.join(RESULTS_DIR, "pareto.png")
        plt.savefig(png, dpi=130, bbox_inches="tight")
        print(f"\n[저장] {png}")
    except Exception as e:
        print(f"\n(matplotlib 없음 → png 생략: {e})")


if __name__ == "__main__":
    main()
