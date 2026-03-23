# adi_ws

> **2026-1 스마트모빌리티종합설계**
> Lightweight VLM-Assisted Task Planning for Autonomous Driving on Scale Car Platform

---

## 개요

LIMO 로봇(AgileX)을 플랫폼을 사용하는 자율주행 종합설계 프로젝트입니다.
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
(개발 중)

### `src/Control` — 제어
(개발 중)

### `src/Moondream` — VLM 장면 이해
경량 Vision-Language Model **Moondream2** (`vikhyatk/moondream2`)를 로컬에서 추론합니다.

| 파일 | 설명 |
|---|---|
| `src/infer_webcam.py` | 웹캠 실시간 추론 — `Space`: 캡처 후 추론, `q`: 종료 |
| `src/infer_image.py` | 단일 이미지 추론 |
| `src/config.json` | 모델 경로, 프롬프트, 웹캠 인덱스 설정 |

```bash
# 웹캠 추론 실행
cd src/Moondream/src
python3.10 infer_webcam.py
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
```bash
cd docker-noetic
```

## 참고

- `ros1/` 디렉토리: LIMO 하드웨어에 기존 탑재되어 있던 ROS 패키지 전체 (참고/보관용)
  → 상세 패키지 설명: [`ros1/README.txt`](ros1/README.txt)
