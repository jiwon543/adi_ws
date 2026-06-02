# 송도 데모 프리플라이트 체크리스트

미션: **lane following** 기본 + `crosswalk_stop → cone_avoidance → obstacle_stop` 순.
구성: **리모**(perception+decision+control, ROS master) ↔ **노트북**(VLM).

> 자동 진단: 노드 다 띄운 뒤 리모에서 `./check_demo.sh limo`, 노트북에서 `./check_demo.sh laptop`.
> 아래는 그 결과를 해석하고 손으로 잡는 순서.

---

## 0. 실행 순서 (반드시 이 순서)

1. **리모**: 센서 드라이버(카메라/라이다/base) 켜짐 확인 → `./run_limo.sh`
   - `run_limo.sh`가 `ROS_IP=192.168.0.4`, master(localhost:11311)로 `limo_bringup.launch` 기동.
2. **노트북**: `~/.bashrc`에 `ROS_MASTER_URI=http://192.168.0.4:11311`, `ROS_IP=<노트북IP>` 확인 → `./run_vlm.sh`
   - 모델 로딩 + warmup에 수십 초. `[VLM] Ready` 뜰 때까지 대기.
3. **리모**: `./check_demo.sh limo` 로 전 토픽 초록불 확인 후 주행 시작.

---

## R1. 🔴 노트북↔리모 ROS 네트워킹 (가장 흔한 전체 실패)

증상: 리모 콘솔에 `[decision] VLM 미연결 — LANE_FOLLOW 유지` 가 5초마다 반복 → **미션 전부 스킵, 차선주행만 함**.

확인:
```bash
# 노트북에서
echo $ROS_MASTER_URI $ROS_IP        # http://192.168.0.4:11311  <노트북IP>
ping 192.168.0.4                    # 리모 응답?
rostopic list | grep camera         # 리모 토픽이 보이면 master 연결 OK
# 리모에서
rostopic echo -n1 /vlm/mission      # 노트북 VLM 결과가 꽂히면 양방향 OK
```
고치기: 양쪽 다 `export ROS_IP=<그 머신의 실제 IP>` (hostname 금지). 같은 서브넷·방화벽 off.

## R2. 🔴 카메라 compressed 토픽

`vlm_node` 와 `camera_lane` **둘 다** `/camera/color/image_raw/compressed` 구독. 없으면 인지 전체 정지.
```bash
rostopic hz /camera/color/image_raw/compressed   # 값이 떠야 함 (보통 15~30Hz)
```
없으면(raw만 발행 시) 리모에서 republish:
```bash
rosrun image_transport republish raw in:=/camera/color/image_raw compressed out:=/camera/color/image_raw
```

## R3. 🟡 콘 검증 임계값 `cone_min_clusters: 4`

`decision_node.yaml`. 1.5m·±0.7m 안에서 동시에 **4개 클러스터** 안 잡히면 콘 회피 미발동(2s 후 차선주행 복귀, 그냥 통과).
```bash
# 콘 깔고 차를 그 앞에 둔 뒤, 한 메시지의 점 개수 확인
rostopic echo -n1 /lidar/clusters | grep -c "x:"
```
동시에 보이는 콘이 2~3개면 → `src/decision/config/decision_node.yaml` 의 `cone_min_clusters` 를 그 수로 낮춤. (재시작 필요)

## R3.5 🟠 콘 회피 조기 종료 (코드 레벨 — 발견된 실제 리스크)

`decision_node.yaml`은 콘을 **1.5m**(`cone_dist_thresh`)에서 트리거하지만
`cone_avoidance_control.yaml`은 **0.7m**(`avoid_dist`) 안 콘만 처리.
→ 트리거 직후 콘이 0.7~1.5m 구간에만 있으면 control이 보는 콘 리스트가 비고,
"정면 클리어" 판정(`cone_clear_sec=1.0s`)이 돌아 **콘에 닿기 전에 `CONE_AVOID_DONE` 조기 발행** 가능
(이후 7s 쿨다운으로 재진입도 막혀 콘 구간을 그냥 통과).

증상: VLM은 cone 찍는데 차가 콘 앞에서 회피 없이 직진해 지나감.

**권장 수정 (택1, 둘 다 config 한 줄 — 재시작만 하면 됨):**
1. (간단) `src/control/config/cone_avoidance_control.yaml` 의 `avoid_dist: 0.7` → **`1.5`** 로 올려
   decision 트리거 거리와 맞춤. 이러면 트리거 즉시 control도 콘을 보고 gap 조향 시작 → 조기 클리어 없음.
   (근접 후진 FSM은 `stop_dist:0.05` 기준이라 영향 없음.)
2. (대안) `src/decision/config/decision_node.yaml` 의 `cone_dist_thresh: 1.5` → **`0.8`** 로 낮춰
   control 처리거리에 맞춤. 단 트리거가 늦어져 회피 여유가 줄어듦.

→ **1번(avoid_dist=1.5) 권장.** 현장에서 콘 한 줄 깔고 한 번만 주행해 확인.

## R4. 🟡 콘 구간 비상정지 오발동

`lidar_emrg_force_dist:0.70 / y:0.20` (decision_node.yaml). LANE_FOLLOW에서 **최우선**이라, 콘 회피가 걸리기 전에 콘이 정면 0.70m·±0.20m 안에 들어오면 비상정지가 대신 걸림.
- 보통 콘 검증(1.5m)이 먼저라 괜찮음. 깨지면: 콘을 정중앙에서 살짝 오프셋해 배치하거나, `lidar_emrg_force_dist` 를 0.5m로 낮춤.

## R5. 🟢 obstacle_stop 동작 방식 (정상)

obstacle 정지는 VLM이 아니라 **R4의 LiDAR 강제경로(0.70m·±0.20m)** 가 담당 → VLM이 obstacle을 못 봐도 멈춤(견고).
정지 후 장애물이 **1초간 사라지면** `EMERGENCY_STOP_DONE` → 차선주행 복귀. 장애물 안 치우면 계속 정지(데모상 의도면 OK).
- `decision_node.yaml`의 `emergency_dist:0.20`은 VLM 검증용 죽은 경로 — 무시해도 됨.

## R6. 🟢 노드 못 찾음 에러

`cannot locate node of type [...]` 뜨면:
```bash
chmod +x src/*/scripts/*.py src/moondream/src/vlm_node.py
cd ~/adi_ws && catkin_make && source devel/setup.bash
```

---

## 미션별 통과 기준 한눈에

| 미션 | 진입 조건 (decision) | 동작 (control) | 복귀(DONE) | 쿨다운 |
|---|---|---|---|---|
| CROSSWALK | VLM=crosswalk_stop **AND** yellow≥4000 (2s내) | 3s 정지 | 3s 후 자동 | 7s |
| CONE_AVOID | VLM=cone_avoidance **AND** 클러스터≥4@1.5m (2s내) | gap 조향 회피 | 정면 클리어 1s | 7s |
| EMERGENCY | 전방 0.70m·±0.20m 클러스터(VLM 불필요) | 정지 유지 | 장애물 사라지고 1s | 7s |
| LANE_FOLLOW | 기본 | Stanley 조향 주행 | — | — |

## 빠른 라이브 모니터 (주행 중 한 터미널)
```bash
rostopic echo /decision/mission        # 미션 전환 실시간
rostopic echo /vlm/mission             # VLM 판단(candidate/raw)
```
