#!/bin/bash
# run_vlm.sh — 로컬(노트북)에서 moondream VLM 노드 실행
# ~/.bashrc에 ROS_MASTER_URI, ROS_IP 설정 전제

set -e

source ~/.bashrc

echo "[vlm] ROS_MASTER_URI = $ROS_MASTER_URI"
echo "[vlm] ROS_IP         = $ROS_IP"
echo "[vlm] roscore 대기 중..."

until rostopic list &>/dev/null; do sleep 1; done

echo "[vlm] 연결 성공. vlm_node 시작..."

source /opt/ros/noetic/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VLM_NODE="$SCRIPT_DIR/src/moondream/src/vlm_node.py"

python3.10 "$VLM_NODE"
