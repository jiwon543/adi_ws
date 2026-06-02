# Trade-off 곡선 1차 결과 (2026-06-02)

> 측정 HW: **이 WSL 머신 / RTX 2060 SUPER 8GB, GPU-sync** (1차 스모크 — 운영 노트북 아님)
> 모델: Moondream2 rev 2025-01-09, FP16 / greedy / input 384
> 데이터: `frames/{cone,obstacle,lane}` 각 30장 (총 90), GT=폴더명, 3클래스
> 원자료: `outputs/token_sweep/run_20260602_135542/` (PyCharm), 곡선 그림 `results/pareto.png`

![pareto](pareto.png)

---

## 1. 한눈 결과

| 세팅 | L_p95 | Acc_bal | 비고 |
|---|---|---|---|
| baseline cap=2 | 540 ms | 35.6% | 가장 빠름·가장 부정확(lane만 맞음) |
| baseline cap=12 (무릎) | 844 ms | 92.2% | 급상승 |
| **baseline cap=16 (동작점)** | **975 ms** | **98.9%** | **천장 도달 최저 cap** |
| baseline cap=50 (운영점) | 1224 ms | 98.9% | 정확도 동일, 더 느림 |
| strict (전 캡) | ~516 ms | 40.0% | 곡선 아님 — 한 점에 붕괴 |

### ★ 결과 ② (교수님 latency 질문에 대한 답)
> **"출력 토큰 캡을 30→16으로 줄여 VLM의 L_p95를 약 20% 낮추면서(1224→975ms) 균형 정확도
> 98.9%를 그대로 유지했다."** 속도는 고정 비용이 아니라 우리가 고르는 손잡이임을 곡선으로 증명.

---

## 2. baseline 곡선의 형태 (클래스별 recall로 설명)

| cap | L_p95 | Acc_bal | cone | obstacle | lane |
|---|---|---|---|---|---|
| 2 | 540 | 35.6% | 6.7 | 0.0 | **100** |
| 3 | 567 | 66.7% | 93.3 | 6.7 | 100 |
| 5 | 628 | 76.7% | 93.3 | 36.7 | 100 |
| 6 | 654 | **46.7%** | 96.7 | 43.3 | **0.0** ← 딥 |
| 12 | 844 | 92.2% | 100 | 76.7 | 100 |
| 16 | 975 | **98.9%** | 100 | 96.7 | 100 |
| 50 | 1224 | 98.9% | 100 | 96.7 | 100 |

- **상승**: 캡이 커질수록 정답 키워드가 답변 안에 들어와 cone→obstacle 순으로 recall 회복.
- **cap=6 딥(lane 0%)**: 6토큰이면 lane 답변이 *"...orange cones..."*까지 닿아 파서가 `cone`으로
  오인 → lane 전멸. cap≥12에서 *"yellow/striped"* 가 나오면 `crosswalk_stop`(우선순위 ↑) → `lane`
  로 회복. = 정의서가 말한 "긴 답변의 거짓양성/오분류"가 토큰 위치 단위로 드러난 것.
- **plateau(cap≥16)**: Acc_bal 98.9% 고정. 남은 오차는 obstacle 30장 중 1장(계통)으로 캡으론 못 고침.
- 즉 정확도=길이의 **역U/계단형**, 지연=길이의 **단조**. → trade-off 성립(정의서 §3 CONCEPT.md).

## 3. strict 프롬프트의 붕괴 (중요)

strict("Answer with one word: cone, obstacle, or clear")는 **거의 모든 장면에 "cone"** 응답:
cone recall 100 / obstacle 20 / lane 0 → Acc_bal 40% 고정, 캡과 무관(답이 1토큰).
→ 라벨 강제는 속도-정확도 맞바꿈이 아니라 **"항상 cone" 퇴화 상태**로 무너진다.
**결론: 이 파서/모델 조합에선 strict 폐기, baseline + 토큰 캡으로 조절이 정답.**

## 4. 속도 분해 재확인
- encode ≈ 415~445 ms (내용 무관 상수, 이 HW 바닥).
- decode ≈ 27 ms/token. → L_total ≈ encode + 27·min(자연길이, cap).
- strict가 ~516ms로 바닥인 이유 = decode가 1토큰뿐.

## 5. 한계 / 다음
- **HW**: 위 수치는 RTX 2060 SUPER(WSL). 운영 노트북에서 다시 떠야 배포용 곡선(정의서 7-1, 2차).
- **T_budget 미정**: (감지거리−안전여유)/주행속도 3요소 확정 후 `pareto.png`에 세로선 긋고
  feasible 중 최고 Acc_bal점을 최종 동작점으로. 현재는 정확도 plateau 진입점(cap=16)을 잠정 동작점.
- **데이터**: 90장(30×3). 증량 시 plateau·딥 안정성 향상. crosswalk는 lane에 흡수(별도 클래스 X).
- **리모 검증(7-5)**: 동작점 cap을 `prompt_vlm_node.json`의 `max_tokens`에 반영해 1회 주행 확인.
- 재현: 스윕 `scripts/test/sweep_tokens.py`, 점수화 `tradeoff_eval/score_profile.py`,
  곡선 `plot_pareto.py`(ASCII) 또는 `scripts/test/plot_curve_png.py`(PNG, venv).
  (참고: WSL→Windows 실행 시 `SWEEP_*` env 오버라이드는 interop로 전달 안 됨 → 풀 설정 실행됨.)
