================================================================================
  ros1/catkin_ws/src 패키지 설명
================================================================================

[ 주요 디렉토리 구조 ]
  ros1/catkin_ws/
  ├── src/      ← 소스 패키지 (아래 목록)
  ├── build/    ← 빌드 산출물 (catkin_make 재생성 가능, 삭제 OK)
  └── devel/    ← 빌드 결과물 (catkin_make 재생성 가능, 삭제 OK)

================================================================================
  패키지별 설명
================================================================================

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  LIMO 로봇 기반 패키지
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[limo_ros] (48M)
  - LIMO 로봇 핵심 드라이버 패키지
  - 하위 패키지: limo_base (하드웨어 통신), limo_bringup (실행 런치), learning_limo (예제)
  - 실제 로봇 구동 시 필수

[limo_description] (175M)
  - LIMO 로봇의 URDF 모델 및 3D 메시(mesh) 파일
  - RViz 시각화 및 Gazebo 시뮬레이션에 사용
  - meshes/ 와 urdf/ 가 용량 대부분 차지

[limo_application-master] (300K)
  - LIMO 공식 애플리케이션 예제 모음
  - 자율주행, 매핑 등 고수준 시나리오 데모

[limo_examples-master] (144K)
  - LIMO 기본 동작 예제 코드 모음

[limo_visions] (496K)
  - LIMO 공식 비전 처리 패키지
  - 레인 감지, 물체 인식 등 카메라 기반 기능

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  인하대 25년 겨울 실습 패키지
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[inha25-winter-ros] (370M, 대부분 .git 히스토리)
  - 인하대 2025년 겨울 방학 LIMO 실습 저장소
  - CATKIN_IGNORE 파일 있음 (빌드에서 제외된 상태)
  - 하위 패키지: limo_mission
    └── scripts/
        - lane_detect_node.py       : 차선 감지 및 추종
        - traffic_light_node.py     : 신호등 인식
        - pedestrian_node.py        : 보행자 감지
        - obstacle_avoid_node.py    : 장애물 회피
        - parking_node.py           : 주차 미션
        - roundabout_node.py        : 로터리(회전교차로) 주행
        - aruco_detector_node.py    : ArUco 마커 감지
        - final_codes/              : 최종 제출 코드
        - custom_node/              : 커스텀 노드

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  센서 드라이버 패키지
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[YDLidar-SDK] (36M)
  - YDLidar 라이다 공식 SDK (C++ 라이브러리)
  - ydlidar_ros_driver의 의존성

[ydlidar_ros_driver] (1.7M)
  - YDLidar 라이다 ROS 드라이버
  - /scan 토픽으로 LaserScan 퍼블리시

[ros_astra_camera] (34M)
  - Orbbec Astra 깊이 카메라 ROS 드라이버
  - RGB + Depth 스트림 제공

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  AR 마커 관련 패키지
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[aruco_ros] (2.4M)
  - OpenCV ArUco 마커 감지 ROS 래퍼
  - 마커 위치/자세 추정 및 TF 퍼블리시

[ar_track_alvar] (2.4M)
  - ALVAR 기반 AR 마커 트래킹 패키지
  - 개별 마커 및 마커 번들 지원

[detect_ar] (44K)
  - ArUco 마커를 감지하여 로봇을 마커 위치로 이동시키는 노드
  - CATKIN_IGNORE 있음 (현재 빌드 제외)
  - src: move_to_ar.cpp, lifter.cpp

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  자율주행 / 네비게이션 패키지
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[agilex_pure_pursuit] (452K)
  - Pure Pursuit 알고리즘 기반 경로 추종 제어기
  - 경로 녹화(record_path) 및 추종(pure_pursuit) 노드 포함

[rrt_exploration] (528K)
  - RRT(Rapidly-exploring Random Tree) 기반 자율 탐색/맵핑
  - Frontier 탐색으로 미지 영역 자동 탐색

[set_nav_point] (40K)
  - 네비게이션 목표 지점을 순서대로 전송하는 유틸리티
  - 다중 웨이포인트 순차 이동 지원

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  기타 커스텀 패키지
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[vision] (144K)
  - 카메라 기반 비전 처리 커스텀 패키지
  - 레인 감지, 신호등/교통표지 인식, 물체 위치 추정
  - RealSense / 일반 카메라 모두 지원
  - scripts: detect_node.py, lane_detection.py 등

[detect_ros] (40K)
  - CATKIN_IGNORE 있음 (현재 빌드 제외)
  - RPY(Roll-Pitch-Yaw) 기반 방향 제어 노드 포함
  - src: lifter.cpp

[lifter_ctr] (272K)
  - 리프터(Lifter) 하드웨어 제어 패키지
  - scripts: lifter_ctr.py
  - CATKIN_IGNORE 있음 (현재 빌드 제외)

[voice] (608K)
  - 음성 녹음 및 STT(Speech-to-Text) 기반 로봇 제어
  - scripts: voice_ctr_node.py, demo_record_voice.py, demo_voice2word.py

================================================================================
  비고
================================================================================

- CATKIN_IGNORE 파일이 있는 패키지는 catkin_make 빌드에서 제외된 상태
  해당 패키지: inha25-winter-ros, detect_ar, detect_ros, lifter_ctr

- build/ 와 devel/ 은 생성된 산출물이므로 삭제 후 catkin_make 로 재생성 가능

================================================================================
