# adi_ws

**2026-1 스마트모빌리티종합설계**
Lightweight VLM-Assisted Task Planning for Autonomous Driving on Scale Car Platform

LIMO(AgileX) 스케일카 플랫폼에서 경량 Vision-Language Model(**Moondream2**)을 운전 보조 판단에 사용하는
ROS Noetic 자율주행 프로젝트입니다. 카메라·LiDAR 인식 → VLM 힌트 + 센서 검증 기반 판단 → 미션별 제어까지
전체 파이프라인을 구현했습니다.

---

## 핵심 아이디어

VLM은 장면을 **언어로 이해**해 다음 미션 후보를 제안하고(예: "콘이 보인다" → `cone_avoidance`),
실제 미션 전환은 **LiDAR/카메라 센서로 한 번 더 검증**한 뒤에만 일어납니다. VLM 단독의 오탐(멀리 있는 콘을
계속 콘이라 함 등)을 센서 게이팅으로 걸러, 경량 VLM을 실차 판단에 안전하게 끼워 넣는 것이 목표입니다.

데모 트랙(송도) 미션 순서: **lane following**(상시) + `crosswalk_stop → cone_avoidance → obstacle_stop`.

---

## 시스템 아키텍처

```
 ┌────────────── 노트북 (VLM 전용) ──────────────┐
 │  vlm_node.py  (Moondream2, GPU)               │
 │   sub: /camera/color/image_raw/compressed     │
 │   pub: /vlm/mission  {candidate, vlm_raw}     │
 └───────────────────────┬───────────────────────┘
                         │ (ROS 멀티마스터, 리모가 master)
 ┌───────────────────────▼──────────── 리모 (LIMO) ────────────────────────┐
 │ Perception                                                              │
 │   camera_lane.py  → /perception/{lateral_offset,heading_error,          │
 │                      curvature,lane_detected,yellow_pixel_count}        │
 │   lidar_clustering.py (/scan→DBSCAN) → /lidar/clusters (PointCloud)     │
 │                                                                         │
 │ Decision                                                                │
 │   decision_node.py  (VLM 힌트 + 센서 검증 FSM) → /decision/mission       │
 │                                                                         │
 │ Control (미션별, 자기 차례에만 /cmd_vel 발행)                              │
 │   lane_control · cone_avoidance_control ·                               │
 │   crosswalk_control · obstacle_stop_control                            │
 │                       └ 완료 시 → /decision/mission_done → Decision      │
 └─────────────────────────────────────────────────────────────────────────┘
```

### 미션 전환 로직 (decision_node)

| 미션 | VLM 힌트 | 센서 검증 (AND) | 동작 | 복귀 조건 |
|---|---|---|---|---|
| `LANE_FOLLOW` | (기본) | — | Stanley 차선 추종 | — |
| `CROSSWALK` | `crosswalk_stop` | 노란픽셀 ≥ `yellow_thresh`(4000) | 3초 정지 | 정지 완료 |
| `CONE_AVOID` | `cone_avoidance` | 1.5m·±0.7m 클러스터 ≥ `cone_min_clusters`(4) | gap 중심 조향 회피 | 정면 경로 클리어 |
| `EMERGENCY_STOP` | `obstacle_stop` *또는* **LiDAR 강제** | 전방 0.70m·±0.20m 클러스터 (VLM 불필요) | 정지 유지 | 장애물 해소 |

- **콘 오탐 방지**: VLM이 멀리 있는 콘을 계속 콘이라 해도, LiDAR 클러스터 수 검증을 통과해야만 회피 진입.
- **횡단보도**: 노란 줄무늬 픽셀 수 임계치로 강제 검증.
- **비상정지**: 좁은 정면(±0.20m)에 길쭉한 클러스터가 잡히면 VLM 없이도 LiDAR가 직접 정지 트리거.
- 미션 완료 후 같은 미션 재진입은 `mission_cooldown`(7초) 동안 차단.

---

## 디렉토리 구조

```
adi_ws/
├── run_limo.sh             ← [리모] 전체 스택 실행 (perception+decision+control)
├── run_vlm.sh              ← [노트북] VLM 노드 실행
├── run_demo.sh             ← [단일 PC] rosbag 재생 데모 (현장 아님)
├── run_demo_v2.sh          ← [단일 PC] rosbag 데모 + 시각화 오버레이 버전
├── check_demo.sh           ← 프리플라이트 자동 진단 (토픽/Hz/네트워크)
├── DEMO_CHECKLIST.md       ← 현장 실행 절차 + 리스크 체크리스트
├── src/
│   ├── perception/
│   │   ├── Camera/         ← camera_lane.py (BEV 차선 인식 + 노란픽셀 카운트)
│   │   └── Lidar/          ← lidar_clustering.py (DBSCAN 클러스터링)
│   ├── decision/           ← decision_node.py (VLM+센서 검증 FSM)
│   │                          demo_viz_node_v2.py (데모 시각화)
│   │                          demo_vlm_mock.py (VLM mock, 디버깅용)
│   ├── control/            ← lane / cone_avoidance / crosswalk / obstacle_stop 제어
│   └── moondream/          ← VLM 모듈
│       ├── src/vlm_node.py            ← 운영 추론 노드
│       ├── src/prompt_vlm_node.json   ← 운영 프롬프트/설정
│       └── src/latency_measure/       ← 지연 측정 스크립트 + 결과 CSV
├── adilimo_ws/             ← LIMO 하드웨어 드라이버 (limo_base, bringup, YDLiDAR, Astra)
└── docker-noetic/          ← ROS Noetic Docker 환경
```

---

## 실행 방법 (현장 데모)

> 센서 드라이버(LIMO base / YDLiDAR `/scan` / Astra 카메라)는 리모에 이미 설치되어 있다고 가정합니다.
> 자세한 절차·트러블슈팅은 [`DEMO_CHECKLIST.md`](DEMO_CHECKLIST.md) 참고.

**1) 리모 (ROS master)**
```bash
./run_limo.sh          # ROS_IP=192.168.0.4, decision/limo_bringup.launch 기동
```

**2) 노트북 (VLM)** — `~/.bashrc`에 `ROS_MASTER_URI=http://192.168.0.4:11311`, `ROS_IP=<노트북IP>` 전제
```bash
./run_vlm.sh           # Moondream2 로딩 후 '[VLM] Ready' 대기
```

**3) 주행 전 진단**
```bash
./check_demo.sh limo       # 리모에서
./check_demo.sh laptop     # 노트북에서
# 핵심 토픽이 모두 초록불 + Hz>0 이면 주행 시작
```

### rosbag 데모 (단일 PC, 영상 촬영용)
```bash
./run_demo.sh /path/to/songdo.bag
```

---

## VLM 모듈 (Moondream2)

운영 노드 `src/moondream/src/vlm_node.py`:
- 모델 `vikhyatk/moondream2` (rev `2025-01-09`), FP16, greedy 디코딩
- `/camera/color/image_raw/compressed` 구독 → ROI 크롭 → `query` → 키워드 우선순위 파싱(`parse_mission`)
- 결과를 `/vlm/mission`(JSON: `candidate`, `vlm_raw`)으로 발행, 추론 주기 `infer_interval`(0.5s)
- 설정: `src/moondream/src/prompt_vlm_node.json` (`prompt`, `max_tokens`, `roi_top_ratio`)

---

## 주요 토픽 레퍼런스

| 토픽 | 타입 | 발행 | 구독 |
|---|---|---|---|
| `/camera/color/image_raw/compressed` | CompressedImage | (센서) | vlm_node, camera_lane |
| `/scan` | LaserScan | (센서) | lidar_clustering |
| `/vlm/mission` | String(JSON) | vlm_node | decision_node |
| `/perception/lateral_offset`·`/heading_error`·`/curvature`·`/lane_detected` | Float32/Bool | camera_lane | lane_control |
| `/perception/yellow_pixel_count` | Int32 | camera_lane | decision_node |
| `/lidar/clusters` | PointCloud | lidar_clustering | decision, cone/obstacle 제어 |
| `/decision/mission` | String | decision_node | 4개 control |
| `/decision/mission_done` | String | control | decision_node |
| `/cmd_vel` | Twist | 미션별 control | LIMO base |

---

## 빌드

```bash
cd ~/adi_ws
catkin_make
source devel/setup.bash
```

의존: ROS Noetic, OpenCV, `cv_bridge`, `scikit-learn`(DBSCAN). VLM 노드는 추가로 PyTorch/transformers(GPU).

---

## Future Work

- **Gazebo 시뮬레이션**: LIMO URDF + autorace world 기반 시뮬레이션 환경(`limo_gazebo_sim`)을 시도했으나
  통합 난이도로 보류. 향후 실차 주행 전 미션 로직을 시뮬레이터에서 검증하는 파이프라인으로 확장 예정.
- 운영 노트북 기준 trade-off 곡선 재측정 및 미션 허용시간(T_budget) 기반 동작점 확정.

---

## 참고

- 현장 실행 상세 절차 및 트러블슈팅: [`DEMO_CHECKLIST.md`](DEMO_CHECKLIST.md)
- Docker 환경: `docker-noetic/`
- LIMO 하드웨어 드라이버 패키지: `adilimo_ws/`
