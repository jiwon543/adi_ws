# adi_ws

**2026-1 스마트모빌리티종합설계**
Lightweight VLM-Assisted Task Planning for Autonomous Driving on Scale Car Platform

---

## 개요

LIMO 로봇(AgileX) 플랫폼을 사용하는 자율주행 종합설계 프로젝트입니다.
카메라 · LiDAR 센서 인식부터 판단, 제어까지의 전체 파이프라인을 ROS Noetic 기반으로 구성합니다.
Docker 환경을 지원하며, 경량 Vision-Language Model(Moondream2)을 이용한 장면 이해 및 태스크 플래닝이 프로젝트 핵심 모듈입니다.

---

## 디렉토리 구조

```
adi_ws/
├── src/                    ← 주 개발 패키지
│   ├── Perception/         ← 인식 (카메라 차선 감지, LiDAR 장애물 감지 등)
│   ├── Decision/           ← 판단 (VLM 판단 기반 미션 상태머신 등)
│   ├── Control/            ← 제어 (경로 제어기)
│   ├── Moondream/          ← VLM 모듈 (Moondream2)
│   └── bringup/            ← 센서 브링업 (LIMO base, YDLiDAR, Astra Camera)
├── ros1/                   ← LIMO 하드웨어 기존 패키지 모음 (참고용)
│   └── catkin_ws/src/      ← 상세 내용: ros1/README.txt 참고
├── docker-noetic/          ← ROS Noetic Docker 환경
│   ├── Dockerfile
│   └── docker-compose.gpu.yaml
├── build/                  ← catkin 빌드 산출물 (재생성 가능)
└── devel/                  ← catkin devel 산출물 (재생성 가능)
```

---

## 패키지 설명

### `src/Perception` — 인식
(개발 중)
| 서브 디렉토리 | 내용 |
|---|---|
| `Camera/` | 카메라 이미지 구독 → 차선 감지 → 차선 중심·곡률 퍼블리시 |
| `Lidar/` | LiDAR 포인트클라우드 처리 |


### `src/Decision` — 판단
(게발 중)
VLM 장면 이해를 통한 추론 결과를 구독, 이에 대한 미션 토픽을 퍼블리시합니다.

### `src/Control` — 제어
(개발 중)
판단에서 나온 토픽을 구독, 차량을 제어합니다.

### `src/Moondream` — VLM 장면 이해
(개발 중)
경량 Vision-Language Model **Moondream2** (`vikhyatk/moondream2`)를 로컬에서 추론합니다. 추론 결과를 Decision에 토픽으로 퍼블리시합니다.

```bash
# 테스트용 웹캠 추론 실행
cd src/Moondream/src/test
python3.10 infer_webcam.py
```

#### VLM 레이턴시 측정 환경 (참고용 — venv 삭제됨)

`src/moondream/src/latency_measure/` 에서 레이턴시 측정 실험 시 사용했던 Python 가상환경(vlm_env) 구성.
재현 시 `python3.10 -m venv vlm_env` 후 아래 버전으로 설치.

| 패키지 | 버전 |
|---|---|
| torch | 2.4.1+cu121 |
| torchvision | 0.19.1+cu121 |
| torchaudio | 2.4.1+cu121 |
| transformers | 4.47.0 |
| accelerate | 1.13.0 |
| huggingface_hub | 0.36.2 |
| numpy | 2.2.6 |

```bash
# 재설치 시
python3.10 -m venv vlm_env
source vlm_env/bin/activate
pip install torch==2.4.1+cu121 torchvision==0.19.1+cu121 torchaudio==2.4.1+cu121 --index-url https://download.pytorch.org/whl/cu121
pip install transformers==4.47.0 accelerate==1.13.0 huggingface_hub==0.36.2 numpy==2.2.6
```

### `src/bringup` — 센서 브링업
LIMO 로봇 실행에 필요한 드라이버 및 런치 파일 모음

| 패키지 | 역할 |
|---|---|
| `limo_bringup` | LIMO 전체 센서 일괄 브링업 런치 |
| `limo_base` | LIMO 하드웨어 통신 드라이버 |
| `ydlidar_ros_driver` | YDLiDAR(Timini) 드라이버 → `/scan` |
| `ros_astra_camera` | Astra(Dabai U3) 깊이 카메라 드라이버 |

```bash
# 전체 센서 브링업
roslaunch limo_bringup limo_bringup.launch
```

---

## 실행 환경

### Docker (권장)

ROS1 Noetic

```bash
cd docker-noetic
```

## 참고

- `ros1/` 디렉토리: LIMO 하드웨어에 기존 탑재되어 있던 ROS 패키지 전체 (참고/보관용)
  → 상세 패키지 설명: [`ros1/README.txt`](ros1/README.txt)

### 가제보 시뮬레이션
(수정 중)

source ~/adi_ws/ros1/catkin_ws/devel/setup.bash

- 리모 urdf + turtlebot world 사용
roslaunch limo_gazebo_sim limo_ackerman.launch \
  world_name:=/opt/ros/noetic/share/turtlebot3_gazebo/worlds/turtlebot3_autorace_2020.world \
  gui:=false


roslaunch limo_gazebo_sim limo_ackerman.launch \
  world_name:=$HOME/adi_ws/ros1/catkin_ws/src/limo_description/worlds/limo_autorace.world \
  x:=0.0 y:=-1.7 \
  gui:=false