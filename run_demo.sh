#!/bin/bash
# run_demo.sh

BAG=${1:-/root/adi_ws/songdo_01.bag}
SETUP="source /opt/ros/noetic/setup.bash && source /root/adi_ws/devel/setup.bash"

PIDS=()

cleanup() {
    echo ""
    echo "[demo] 종료 중..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    pkill -f "roscore"    2>/dev/null
    pkill -f "roslaunch"  2>/dev/null
    pkill -f "rosbag"     2>/dev/null
    pkill -f "rqt_image"  2>/dev/null
    pkill -f "vlm_node"   2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "[1] roscore..."
bash -c "$SETUP && roscore" &
PIDS+=($!)
sleep 3

echo "[2] demo_v2.launch..."
bash -c "$SETUP && roslaunch decision demo_v2.launch" &
PIDS+=($!)
sleep 4

echo "[3] VLM node 시작 (모델 로딩 대기)..."
bash -c "$SETUP && cd /root/adi_ws/src/moondream/src && python3.10 vlm_node.py" &
PIDS+=($!)

echo "    vlm_infer 노드 등록 대기 (최대 3분)..."
for i in $(seq 1 180); do
    bash -c "$SETUP && rosnode list 2>/dev/null | grep -q vlm_infer" && break
    sleep 1
    echo -n "."
done
echo ""
echo "    VLM 준비 완료!"

echo "[4] bag 재생: $BAG"
bash -c "$SETUP && rosbag play -l $BAG --clock" &
PIDS+=($!)
sleep 2

echo "[5] rqt_image_view..."
bash -c "$SETUP && rqt_image_view /demo/viz_v2" &
PIDS+=($!)

echo ""
echo "종료: Ctrl+C"
wait
