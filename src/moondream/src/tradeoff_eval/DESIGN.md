# VLM 속도–정확도 Trade-off 평가 · 설계 문서

> 대상: `src/moondream` (Moondream2 VLM 모듈)
> 근거 문서: 「VLM 속도–정확도 Trade-off 평가 정의서」 (실험 전 단계)
> 작성 목적: 정의서의 평가식을 **실제로 돌릴 수 있는 코드/실험 절차**로 옮기기 위한 설계.
> 상태: 설계 단계 (코드 작성 전)

---

## 0. 한 줄 요약

세팅 = (프롬프트) × (max_tokens 캡) 한 점마다 **균형 정확도 Acc_bal**(세로축)과
**p95 추론시간 L_p95**(가로축)을 뽑아 trade-off 곡선을 그리고,
미션 허용시간 `T_budget` 안에서 가장 정확한 동작점을 고른다.

---

## 0.5 실측 데이터 (이미 확보됨 · 이 WSL HW 기준)

PyCharm 프로젝트(`C:\Users\jiwlee\PycharmProjects\Moondream`)에 **속도축 측정이 이미 되어 있다.**
`scripts/test/vlm_latency_profiler.py` (encode/decode 분리, GPU-sync) 결과 `outputs/latency_profile/`:

| 항목 | 실측 (RTX 2060 SUPER, GPU-sync) | 의미 |
|---|---|---|
| `encode_ms` | **422~428 ms (CV≈1%)** | 정의서 가설1 실증: 이미지 처리=내용 무관 상수. 이 HW의 **고정 바닥 ~425ms** |
| `decode_ms` | 13토큰→~383ms, 18토큰→~500ms | 정의서 가설2 실증: decode ∝ 토큰 수 |
| 토큰당 시간 | **≈ 26~27 ms/token** | 정의서 회귀식 `27·tok+30`과 일치 (단 +30 자리에 encode≈425ms가 붙음) |
| total | cone 800~960ms (max_tokens 30/50) | 운영 노트북(530~740ms)보다 느림 → HW 종속 재확인 |

> **결정적 관찰**: 프롬프트 "one short sentence"의 자연 답변 길이가 **~13~18토큰**이라
> `max_tokens=30`과 `50`의 출력이 **동일**(캡이 안 걸림). → 의미 있는 스윕 구간은 **cap ≲ 20**.
> 종합: `L_total ≈ 425(encode) + 27·min(자연길이, cap) (decode)` [ms, 이 WSL HW].

---

## 1. 정의서 ↔ 기존 코드 매핑

| 정의서 항목 | 기존 자산 | 갭(새로 만들 것) |
|---|---|---|
| 6.1 파서 (raw 문장 → 노드) | `vlm_node.py : parse_mission()` | 노드 미션 → 정의서 클래스 라벨 매핑 추가 |
| 6.1 정답(GT) = 폴더명 | (없음) `simple/`·`complex/`만 존재 | `cone/crosswalk/obstacle/none` GT 데이터셋 구축 |
| 6.2 정확도 Acc_bal / Acc_w | (없음) | 클래스별 recall·균형정확도·안전가중 계산 |
| 6.3 속도 L = t_encode + t_decode | **PyCharm `vlm_latency_profiler.py`** (encode/decode 분리·GPU-sync·실측완료) + `latency_measure/vlm_image_latency.py` | p95 계산만 추가하면 됨 |
| 7-3 GT 데이터셋 | **PyCharm `frames/{cone,obstacle,lane}` 각 30장** | `lane`→`none` 매핑, `crosswalk` 폴더 추가 수집 |
| 6.3 bimodal·p95 | 중간발표 `latency/hist_vlm.csv` (513~690ms, 530대/620대 2덩어리) | p95/mean/max 산출 로직 |
| 6.3 회귀식 L≈27·tok+30 | 기존 측정값(노트북) | 측정 HW에서 재추정 |
| 6.4 T_budget·동작점 | (없음) | feasible 필터 + argmax Acc_bal 선정 |
| 6.5 Pareto 곡선 | (없음) | 곡선 플롯 + T_budget 세로선 + 동작점 표시 |
| 세팅 축 (프롬프트) | `prompt_vlm_node.json`, `vlm_test/prompt*.json` | 프롬프트 집합으로 정리 |
| 세팅 축 (길이 제어) | `vlm_image_latency.py`의 `--max-tokens` | max_tokens 촘촘 스윕(step=1) |

**핵심 베이스**: `vlm_image_latency.py`는 이미지 셋 × max_tokens 속도 스윕을 이미 한다(raw CSV에 `answer`까지 저장). 여기에 **정확도 축**과 **p95·곡선·동작점**을 더하면 평가 하니스가 된다.

---

## 2. 측정 하드웨어 (정의서 7-1)

곡선 수치는 HW 종속이라, **배포용 곡선은 운영 HW에서 뜬 것만 유효**하다.

| 단계 | HW | 목적 |
|---|---|---|
| 1차 (스모크) | 이 WSL 머신 (RTX 2060 SUPER 8GB) | 파이프라인·식·CSV·플롯이 끝까지 도는지 검증 |
| 2차 (본실험) | 운영 노트북 | 실제 리모에 올릴 곡선·동작점 산출 |

> 주의: 1차 WSL 수치는 **상대 비교/파이프라인 검증용**이지 동작점 확정용이 아니다.
> 동작점(6.4)과 회귀식(6.3)은 반드시 2차 노트북 수치로 확정한다.

### 2.1 실행 환경 (전제)
- 시스템 파이썬 3.14, `torch`/`transformers` **미설치**(venv 삭제됨). GPU는 사용 가능.
- 재현: `python3.10 -m venv vlm_env` 후 README의 핀 버전 설치
  (torch 2.4.1+cu121 / transformers 4.47.0 / accelerate 1.13.0 / huggingface_hub 0.36.2 / numpy 2.2.6).
- 디코딩은 **greedy 고정**(정의서 6.3): moondream `model.query(...)`는 기본 greedy. 재현성·분산 제거 목적.

---

## 3. 세팅 축 정의 (정의서 6, 7-2)

곡선의 한 점 = 하나의 세팅 = **(프롬프트) × (max_tokens 캡)**.

### 3.1 프롬프트 집합
| 이름 | 성격 | 출처 |
|---|---|---|
| `baseline` | open-ended, "한 문장으로 묘사" (정확↑·김) | `prompt_vlm_node.json` |
| `strict` | 라벨 한 단어만 강제 (빠름·헛소리↑) | 신규 작성 |
| `gated` (선택) | 거리 게이팅("가까울 때만 보고") | `vlm_test/prompt_v1.json` |

→ 곡선의 양 끝(자유 문장 ↔ 라벨 강제)을 프롬프트로도 잡아준다.

### 3.2 max_tokens 캡 스윕 — **범위 근거**

> 결정: **min=2, max=24, step=1** (23개 점) + plateau 확인용 {30, 50} 2점.
> (정의서가 예시한 10/20/30 대신, 실측 자연길이 13~18에 맞춰 무릎 구간을 촘촘히 덮음.)

근거 (§0.5 실측 기반):
1. **디코딩 시간은 실제 생성 토큰 수 = `min(자연 답변 길이, 캡)` 에 비례**한다.
   따라서 곡선 점은 `캡 < 자연 답변 길이`인 구간에서만 움직이고,
   캡이 자연 길이를 넘으면 greedy가 EOS에 먼저 닿아 **Acc·L 둘 다 평탄화(plateau)** 된다.
   → 실측에서 cap30=cap50 답변 동일(18토큰)로 이미 확인됨.
2. **상한 24**: baseline 프롬프트 자연 길이가 실측 ~13~18토큰. 24면 그 plateau를 살짝 넘겨
   "더는 안 변함"을 곡선으로 보여줄 수 있다. {30,50}는 plateau 재확인용 앵커.
   (정의서가 든 "30"은 이 프롬프트에선 이미 평탄 구간이라 곡선이 안 움직임 → 무릎은 더 왼쪽.)
3. **하한 2**: 캡=1이면 첫 토큰만 나와(거의 "A"/"There" 류 filler) 파서가 라벨을 못 만든다 → 퇴화점.
   캡=2가 content word가 처음 등장할 수 있는 실질 바닥. (원하면 1을 "절대 바닥 앵커"로 추가.)
4. **step=1**: 사용자 요청. 토큰 간격을 촘촘히 봐 곡선의 무릎(knee)을 정밀하게 잡는다.

> 프롬프트별 자연 길이가 다르므로(strict는 더 짧음) 상한은 프롬프트마다 자동 조정 가능
> — greedy-prefix(§6)로 한 번 cap=50 생성 후 잘라 보면 각 프롬프트 plateau가 데이터로 드러난다.

> 비용 주의: 39캡 × (예: 195장) × repeat 는 매우 큼. → §6의 효율 설계로 1회 생성에서 전 캡을 복원한다.

---

## 4. 정확도 축 (정의서 6.1–6.2)

### 4.1 기본 단위 (이미지 한 장)
```
GT(이미지)   = 클래스 폴더명 ∈ {cone, crosswalk, obstacle, none}
raw          = model.query(...)["answer"]
mission      = parse_mission(raw)               # 기존 vlm_node.py 재사용
pred_class   = MISSION_TO_CLASS[mission]        # 신규 매핑
correct_i    = 1 if pred_class == GT else 0
```

### 4.2 미션 → 클래스 매핑 (신규) — **3클래스 기준**

> **클래스 결정 (2026-06-02)**: 평가는 **3클래스 {cone, obstacle, lane}** 로 간다.
> `crosswalk`/`none`은 별도 클래스로 두지 않는다. 이유: 노란색 횡단보도 마킹이 `lane`
> 프레임 안에 섞여 나타나고, "얼마나 근접했나"로 crosswalk를 분리하는 건 주관적 라벨이라
> 라벨 노이즈가 됨. `lane`이 곧 "주행 가능한 배경(=clear/none)" 클래스 역할을 겸한다.

| parse_mission 출력 | 평가 클래스(3클래스) |
|---|---|
| `obstacle_stop` | `obstacle` |
| `cone_avoidance` | `cone` |
| `crosswalk_stop` | `lane` (횡단보도 마킹은 lane에 포함) |
| `normal_drive` | `lane` (빈 도로 = 배경) |

> 파서 우선순위(obstacle > crosswalk > cone > normal)는 `parse_mission`에 이미 구현됨. 그대로 쓰고,
> 출력 미션만 위 표로 3클래스에 접는다. (`crosswalk_stop`/`normal_drive` 둘 다 → `lane`)

### 4.3 지표
```
Acc      = Σ correct_i / N                         # 단순 전체 정확도(참고)
Recall_c = (클래스 c 맞은 수) / (클래스 c 전체 수)
Acc_bal  = (Σ_c Recall_c) / C                      # ★ 주 지표 (클래스 불균형 보정)
Acc_w    = (Σ_c w_c·Recall_c) / (Σ_c w_c)          # 보조: 안전 가중
```
- 주 지표 = **Acc_bal** (C=3: cone/obstacle/lane). 현재 데이터는 30/30/30 균형이지만,
  실주행 분포가 불균형해질 수 있으니 균형정확도를 기본으로 둔다.
- 안전 가중 기본값(제안): `w_obstacle=3, w_cone=2, w_lane=1` — 놓치면 위험한 순.
  (값은 설정 파일로 빼서 쉽게 바꿈.)
- 혼동행렬(3×3)도 함께 저장 → 어떤 클래스를 무엇으로 헷갈리는지 진단.
  (예상: 긴 답변에서 `lane`이 주변부 cone을 언급해 `cone`으로 오분류 → lane recall↓.)

### 4.4 데이터셋 (정의서 7-3) — **3클래스, 각 30장 확보**

PyCharm `frames/`에 GT 라벨 폴더가 있다(주행 영상 프레임 추출, 0.5s 간격):

| 폴더 | 장수 | 평가 클래스 | 비고 |
|---|---|---|---|
| `frames/cone` | 30 | `cone` | |
| `frames/obstacle` | 30 | `obstacle` | toy car/소형 차량 장애물 |
| `frames/lane` | 30 | `lane` | 빈 도로 + 노란 횡단보도 마킹 포함(배경/none 역할) |

- crosswalk는 **별도 클래스로 안 둠**(§4.2 결정). → 데이터 추가 수집 불필요, 현 30/30/30로 진행.
- greedy라 이미지당 1회 추론으로 정확도 확정(repeat은 속도 안정화용).
- `frames/test*.jpg`(3장)는 라벨 미상 → 평가 제외.
- 증량(클래스당 더 많은 프레임)은 선택. 현재로도 곡선·동작점 산출 가능.

---

## 5. 속도 축 (정의서 6.3)

```
L_i = t_encode + t_decode  [ms]    # encode_image + query(=decode)
```
- **GPU 동기화 ON**으로 측정(`torch.cuda.synchronize()` 전후). 정의서 6.3이 명시.
  → 기존 `vlm_image_latency.py`의 기본값은 legacy_no_sync(ROS 재현용)이므로 `--sync-cuda` 켠다.
- `t_encode`는 장면별 거의 상수(정의서 1번 발견). encode/decode를 **분리 기록**해 이를 데이터로 재확인.
- 대표값: **L_p95**(주). mean·max 병기.
  - p95는 한 세팅의 전 이미지(×repeat) 표본을 정렬한 95퍼센타일.
- **repeat**: greedy면 답변(=토큰 수)이 결정적이라 decode 시간은 HW 지터만 변동.
  → 정확도엔 repeat 불필요(1패스), **속도 표본 안정화용으로만** repeat(예: 5).
- 회귀식 `L ≈ a·(생성 토큰 수) + b` 를 측정 HW에서 재적합(기존 노트북 기준 27·tok+30).

---

## 6. 효율 설계 — 1회 생성에서 전 캡 복원 (★ 핵심)

step=1 × 39캡 × 데이터셋 × repeat 를 캡마다 재추론하면 수 시간~십수 시간. 두 가지로 줄인다.

1. **encode 1회 재사용**: `t_encode`는 캡과 무관(장면만). 이미지당 `encode_image` 한 번 → 모든 캡이 공유.
2. **greedy prefix 성질**: greedy로 캡=40까지 한 번 생성하면, 캡=k의 출력은 그 토큰열의 **앞 k개 prefix**다.
   → **정확도는 캡=max 1회 생성**에서 모든 캡을 복원(앞 k토큰 디토크나이즈 → parse).
   → **속도는 생성 중 per-step decode 시간을 누적 기록**해, 캡 k의 L = t_encode + Σ_{i≤k} decode_step_i.

즉 **(프롬프트, 이미지)당 1회 생성**으로 전 캡의 (정확도, 속도)를 동시에 얻는다.

- 구현 경로 A(효율): moondream의 저수준 generate로 내려가 token id열 + per-step 시간 확보.
  query API가 이를 안 열어주면 모델 remote code를 살짝 감싼다.
- 구현 경로 B(단순·fallback): 기존 방식대로 캡마다 `query` 재호출(느리지만 확실). 1차 스모크는 B의 축소판(step 큰 캡 몇 개·소수 이미지)으로 검증하고, 본실험은 A로.

> 1차 스모크는 경로 B + 소규모(이미지 5~10, 캡 {5,10,20,30})로 "끝까지 도는지"만 본다.

---

## 7. 동작점 선정 (정의서 6.4)

```
T_budget = (감지거리 − 안전여유) / 주행속도        # 로봇 물리에서 (입력 필요)
feasible : L_p95(세팅) ≤ T_budget
best     = argmax Acc_bal  over feasible           # 동률이면 더 빠른 쪽
```
- **입력 TODO**: 감지거리(카메라/LiDAR 유효 인식거리), 안전여유, 주행속도 → 팀 확정 필요.
- 선정 세팅을 실제 리모에 **1회 검증**(정의서 결과 ③).

---

## 8. 산출물 (정의서 6.5, 결과 ①②③)

1. `raw_*.csv` — (prompt, image, cap, repeat) × {t_encode, t_decode, L, raw, pred_class, GT, correct}
2. `setting_summary_*.csv` — (prompt, cap) × {Acc, Acc_bal, Acc_w, per-class recall, L_p95, L_mean, L_max}
3. `pareto.png` — x=L_p95(ms, 왼쪽=빠름), y=Acc_bal(%), Pareto front, T_budget 세로선, 동작점 ★
4. `confusion_*.csv` — 세팅별 4×4 혼동행렬
5. **결과 ② 한 문장**: "출력 길이를 X→Y 토큰으로 줄여 L_p95를 ○○% 낮추면서 Acc_bal은 유지" — 곡선에서 자동 추출.

---

## 9. 산출 스크립트 구조 (예정)

```
tradeoff_eval/
├─ DESIGN.md              ← (이 문서)
├─ config.yaml            ← 프롬프트 집합, 캡 범위, w_c, T_budget 입력, 경로
├─ eval_tradeoff.py       ← 스윕 실행: (prompt,image)당 1생성 → raw CSV
├─ metrics.py             ← parse→class 매핑, Acc_bal/Acc_w, p95, 회귀
├─ plot_pareto.py         ← raw/summary → pareto.png + 결과② 문장
└─ dataset/               ← cone/ crosswalk/ obstacle/ none/  (선행 구축)
```
- `parse_mission`은 `vlm_node.py`에서 import(중복 구현 금지).

---

## 10. 진행 순서 (정의서 7)

- [x] 7-1 측정 HW 확정 — 1차 WSL(RTX 2060S) → 2차 노트북
- [x] 7-2 파라미터 boundary — 프롬프트 {baseline, strict, (gated)}, max_tokens 2~40 step1, greedy
- [~] 7-3 GT 데이터셋 — cone30/obstacle30/lane(none)30 **확보**, crosswalk(45)·증량은 병행
- [ ] 코드: `metrics.py`(정확도 신규) → `eval_tradeoff.py`(profiler 확장) → `plot_pareto.py`
- [ ] venv(vlm_env) 구성 + WSL 1차 스모크 (3클래스 30장으로)
- [ ] 효율 경로 A 구현 후 노트북 본실험 → 곡선·동작점
- [ ] 7-5 동작점 리모 검증

> **속도축은 PyCharm `vlm_latency_profiler.py`로 이미 검증됨** → 신규 작업의 무게중심은 **정확도축**(parse→class, Acc_bal)과 **곡선/동작점**.

## 11. 미결 입력 (팀 확정 필요)
- `crosswalk` 데이터 0장 → 1차는 3클래스(cone/obstacle/none)로 곡선 검증, crosswalk는 수집 후 합류
- 안전 가중치 `w_c` 최종값
- `T_budget` 3요소(감지거리·안전여유·주행속도)
- 효율 경로 A에서 moondream query API가 token id열/per-step 시간을 노출하는지(미확인 → 본실험 전 확인)
- 운영 노트북 사양(2차 본실험 곡선의 기준 HW)
