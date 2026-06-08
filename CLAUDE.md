# CLAUDE.md — adi_ws 기술 핸드오프

> 이 문서는 **노트북(VLM 머신)에서 작업하는 Claude**를 위한 프로젝트 컨텍스트다.
> 송도 트랙 최종 데모를 안정적으로 성공시키는 것이 목표. 사람용 README는 `README.md`,
> 현장 실행 절차는 `DEMO_CHECKLIST.md`, 자동 진단은 `check_demo.sh` 참고.

---

## 1. 프로젝트 한 줄 요약

LIMO(AgileX) 스케일카에서 경량 VLM **Moondream2**가 전방 장면을 **언어로 서술**하면,
키워드 파서가 미션 후보로 바꾸고, **LiDAR/카메라 센서 검증을 통과해야만** 미션이 전환되는
ROS Noetic 자율주행 시스템. VLM 단독 오탐을 센서 게이팅으로 거른다.

미션(송도): 상시 **lane following** + 순서대로 `crosswalk_stop → cone_avoidance → obstacle_stop`.

---

## 2. 배포 구조 — 2대 머신, ROS 멀티마스터

| 머신 | 역할 | 실행 | ROS |
|---|---|---|---|
| **리모(LIMO)** | perception + decision + control + 센서드라이버 | `./run_limo.sh` | **master**, `ROS_IP=192.168.0.4` |
| **노트북(여기)** | VLM 추론만 | `./run_vlm.sh` | client, `ROS_MASTER_URI=http://192.168.0.4:11311`, `ROS_IP=<노트북IP>` |

- 노트북은 리모의 카메라 토픽을 받아 추론하고 결과를 리모로 되돌린다. **양방향 토픽이 흘러야 함.**
- 노트북 `~/.bashrc`에 `ROS_MASTER_URI`/`ROS_IP`가 IP 형태로 설정돼 있어야 한다(hostname 금지).

---

## 3. 데이터 흐름 (토픽 계약 — 검증 완료)

```
[노트북] vlm_node.py
   sub /camera/color/image_raw/compressed (CompressedImage)
   pub /vlm/mission (std_msgs/String, JSON: {candidate, vlm_raw, vlm_publish_time})
                              │
[리모] decision_node.py  ─────┤ sub /vlm/mission, /lidar/clusters, /perception/yellow_pixel_count,
                              │     /decision/mission_done
                              └ pub /decision/mission (String: LANE_FOLLOW|CONE_AVOID|CROSSWALK|EMERGENCY_STOP)

[리모] camera_lane.py  sub /camera/color/image_raw/compressed
        pub /perception/{lateral_offset,heading_error,curvature,lane_detected,yellow_pixel_count,center_point_px,bev,debug}
[리모] lidar_clustering.py  sub /scan(LaserScan) → DBSCAN → pub /lidar/clusters(PointCloud, x=전방 m, y=좌+우-)

[리모] control(4종)  sub /decision/mission (+ 센서)  → pub /cmd_vel(Twist), /decision/mission_done(String)
        lane_control / cone_avoidance_control / crosswalk_control / obstacle_stop_control
```

`candidate` 값: `normal_drive | cone_avoidance | crosswalk_stop | obstacle_stop`
→ decision의 `VLM_MAP`이 미션 상태로 변환.

**중요**: 4개 control 노드가 모두 `/cmd_vel`을 발행하지만, **자기 미션일 때만** 발행한다.
`lane_control`은 미션 전환 순간 zero Twist 한 번만 쏘고 침묵 → 충돌 없음.
검증(_VRF) 중에는 decision이 `LANE_FOLLOW`를 계속 발행해 차선 추종을 유지한다.

---

## 4. 미션 전환 FSM (decision_node.py)

VLM 힌트(필요)와 센서 검증(AND)을 모두 만족해야 미션 진입. 완료 후 `mission_cooldown`(7s) 재진입 차단.

| 미션 | VLM 힌트 | 센서 검증 (AND) | 동작(control) | 복귀(DONE) |
|---|---|---|---|---|
| CROSSWALK | crosswalk_stop | `yellow_pixel_count ≥ yellow_thresh(4000)` | 3초 정지 | 정지 완료 |
| CONE_AVOID | cone_avoidance | 1.5m·±0.7m 클러스터 ≥ `cone_min_clusters(4)` | gap 중심 조향 회피 | 정면 경로 클리어 1s |
| EMERGENCY_STOP | obstacle_stop *또는* **LiDAR 강제** | 전방 0.70m·±0.20m 클러스터 (VLM 불필요) | 정지 유지 | 장애물 해소 1s |
| LANE_FOLLOW | (기본) | — | Stanley 차선 추종 | — |

**핵심 설계 포인트 (디버깅 시 반드시 인지):**
- **비상정지는 VLM이 아니라 LiDAR가 트리거**한다. LANE_FOLLOW에서 `lidar_emergency_close()`
  (0.70m·±0.20m, 좁은 정면)가 **최우선·무조건** 검사 → VLM이 obstacle을 못 봐도 멈춘다.
- decision의 VLM-emergency 검증 경로(`emergency_dist:0.20`)는 너무 좁아 **사실상 죽은 코드**.
  즉 **VLM의 obstacle_stop 힌트는 실질적으로 무동작**. → 파서에서 obstacle 키워드를 좁혀도 손해 없음.
- 따라서 VLM이 실제로 작동시키는 미션은 **cone_avoidance와 crosswalk_stop 둘뿐**.

---

## 5. VLM 노드 (노트북에서 돌리는 핵심)

파일: `src/moondream/src/vlm_node.py`, 설정: `src/moondream/src/prompt_vlm_node.json`
- 모델 `vikhyatk/moondream2` rev `2025-01-09`, FP16, greedy, GPU(`device_map cuda`)
- 흐름: `/camera/...compressed` → ROI 크롭(`roi_top_ratio:0.2`) → `encode_image` → `query`
  → `parse_mission()` 키워드 파싱 → `/vlm/mission` 발행. 추론 주기 `infer_interval:0.5s`.
- 시작 시 warmup 1회(수십 초 모델 로딩 후). `[VLM] Ready` 로그 후 동작.

### 5.1 프롬프트/파서 설계 근거 (변경 금지 사유 포함)

- **closed-set 강제 금지**: "Choose one: crosswalk/cone/obstacle/lane" 식으로 라벨을 강제하면
  Moondream2가 할루시네이션으로 붕괴(실측: strict 프롬프트 → 거의 모든 장면 "cone", Acc 40% 고정).
  → **open-ended 서술 + 키워드 파서** 구조를 반드시 유지.
- **현재 프롬프트**(최적화 적용본, `prompt_vlm_node.json` 기준):
  ```
  You are looking forward through the windshield of a small self-driving car.
  In one short sentence, describe what is on the road directly ahead of you.
  ```
  - 이전 버전의 "Describe **the hazard**"는 위험물 존재를 전제해 빈 차선서도 위험물을 발명 → 제거함.
  - 빈 차선 응답("clear road" 등)은 파서 `clear_phrases`가 normal_drive로 매핑해 처리.
- **파서**(`MISSION_RULES`, 우선순위 obstacle > crosswalk > cone):
  - obstacle `must_any`에서 포괄어 `object/vehicle/toy` 제거 → cone/crosswalk 가림 방지
    (obstacle 힌트는 무동작이므로 좁혀도 안전).
  - obstacle `must_not`: cone 계열(콘과 구분).
  - 키워드 튜닝 시: **cone은 "cone/cones/orange marker/traffic cone", crosswalk는
    "crosswalk/crossing/zebra/yellow stripe/striped/yellow line"** 가 핵심. 이 둘만 잘 잡히면 데모 성립.
- **`max_tokens:12` (확정)**: 지연-정확도 trade-off 실험(90장, 2~50 범위 스윕)으로 결정한 최적값.
  - 12 미만: 답변 잘림 → crosswalk 파싱 실패("striped" 같은 후미 키워드 미생성).
  - 12 초과: 정확도 추가 이득 없이 지연만 증가(디코더가 토큰 수에 선형 비례).
  - 실측 지연(500샘플): 총 631ms = 추론 576ms(91%) + 네트워크 55ms. 이중봉 분포 → 디코더가 변동 주요인.
  - ROSBAG ROS 파이프라인 검증 완료. 상세: `presentation/` 슬라이드 11~13.

---

## 6. 노트북에서의 표준 작업 절차

```bash
# 0) 환경 확인
echo $ROS_MASTER_URI $ROS_IP        # http://192.168.0.4:11311  <노트북IP(아님: hostname 금지)>
ping 192.168.0.4                    # 리모 응답?
rostopic list | grep camera         # 리모 토픽 보이면 master 연결 OK

# 1) VLM 실행 (리모가 먼저 떠 있어야 함)
./run_vlm.sh                        # '[VLM] Ready' 대기

# 2) VLM 출력 라이브 확인 (가장 중요한 디버깅 창)
rostopic echo /vlm/mission          # candidate + vlm_raw(모델 원문) 확인
                                    # 콘 앞 → candidate: cone_avoidance, 횡단보도 → crosswalk_stop 나와야 함

# 3) 전체 진단
./check_demo.sh laptop
```

VLM이 엉뚱하게 찍으면: `vlm_raw`(모델 원문)를 먼저 보고 → 원문은 맞는데 candidate가 틀리면
**파서 키워드** 문제(`vlm_node.py` MISSION_RULES), 원문 자체가 틀리면 **프롬프트/ROI/조명** 문제.

---

## 7. 알려진 리스크 (우선순위순) — 상세는 DEMO_CHECKLIST.md

- **R1 🔴 ROS 네트워킹**: 노트북↔리모 `ROS_IP` 미설정/hostname → VLM 토픽 안 흐름.
  증상: 리모에 `[decision] VLM 미연결 — LANE_FOLLOW 유지` 반복, 미션 전부 스킵.
- **R2 🔴 카메라 compressed 토픽**: `/camera/color/image_raw/compressed` 없으면 VLM·차선 둘 다 정지.
  `rostopic hz`로 확인. 없으면 리모서 `image_transport republish`.
- **R3 🟡 `cone_min_clusters:4`**: 동시에 4개 클러스터 안 잡히면 콘 회피 미발동. 현장 실측 후 조정.
- **R3.5 🟠 (수정 완료)**: cone_control `avoid_dist`를 0.7→1.5로 올려 decision 트리거(1.5m)와 정합
  (콘 조기 클리어 방지). 현장 1회 검증 권장.
- **R4 🟡 콘 구간 비상정지 오발동**: 콘이 정면 0.70m·±0.20m 들면 비상정지가 콘회피보다 먼저 걸릴 수 있음.
- **R5/R6 🟢**: VLM-emergency 죽은 경로(무해), 노드 못 찾으면 `chmod +x` 후 `catkin_make`.

---

## 8. 파일 지도

```
run_limo.sh                               ← [리모] 전체 스택 실행
run_vlm.sh                                ← [노트북] VLM 노드 실행
run_demo.sh / run_demo_v2.sh              ← [단일 PC] rosbag 재생 데모
check_demo.sh / DEMO_CHECKLIST.md         ← 진단/현장절차

src/moondream/src/vlm_node.py             ← ★VLM 노드(파서 포함)
src/moondream/src/prompt_vlm_node.json    ← ★프롬프트/설정
src/moondream/src/latency_measure/        ← 지연 측정 스크립트 + CSV 결과
presentation/                             ← 종설 최종발표 슬라이드 PNG (슬라이드 6~13)

src/decision/scripts/decision_node.py     ← 미션 FSM
src/decision/scripts/demo_viz_node_v2.py  ← 데모 시각화 (run_demo_v2.sh에서 사용)
src/decision/scripts/demo_vlm_mock.py     ← VLM 없이 테스트할 때 쓰는 mock
src/decision/config/decision_node.yaml    ← 검증 임계값(yellow_thresh, cone_min_clusters, lidar_emrg_*)
src/decision/config/vlm_mock.yaml         ← mock 노드 설정

src/perception/Camera/scripts/camera_lane.py     ← 차선+노란픽셀
src/perception/Lidar/scripts/lidar_clustering.py ← DBSCAN 클러스터
src/control/scripts/{lane,cone_avoidance,crosswalk,obstacle_stop}_control.py
src/control/config/*.yaml                 ← 제어 파라미터
```

## 9. 작업 원칙

- 데모 직전이다. **동작 검증된 제어 로직·튜닝값을 임의로 바꾸지 말 것.** 변경은 근거를 명시하고
  config 한 줄 단위로, 되돌리기 쉽게. 실차 거동에 영향 주는 변경은 사용자 확인 후.
- 노트북에선 모델 추론 검증이 핵심. 실제 GPU 추론을 돌려 `vlm_raw`를 눈으로 보고 판단.
- 토픽 이름·메시지 타입은 위 계약을 진실원본으로. 바꾸면 양 끝(발행/구독) 모두 맞출 것.
```
