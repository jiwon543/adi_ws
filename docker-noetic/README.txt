# 1. X11 허용
xhost +local:docker

# 2. 빌드
cd ~/adi_ws/docker-noetic
(2-1.🖥️  GPU 없는 환경)
docker compose up -d --build
(2-2.🎮 GPU 있는 환경)
docker compose -f docker-compose.yaml -f docker-compose.gpu.yaml up -d --build

# 3. 컨테이너 접속
docker exec -it adi-noetic-dev bash

# 4. vscode컨테이너 접속 연동
 4-1. Ctrl+Shift+P 
 4-2. Dev Containers: Attach to Running Containers
 4-3. File -> Open Folder -> /roo/adi_ws 열기

# (기타) adi-noetic-dev 컨테이너만 중지 및 삭제
docker rm -f adi-noetic-dev

# (기타) runtime에 nvidia 안붙을 경우
docker context use default