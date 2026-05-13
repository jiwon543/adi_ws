#!/bin/bash
# run_limo.sh — limo 전체 스택 실행 (리모에서 실행)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/noetic/setup.bash
source "$SCRIPT_DIR/devel/setup.bash"

export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=192.168.0.4

echo "[limo] ROS_MASTER_URI = $ROS_MASTER_URI"
echo "[limo] ROS_IP         = $ROS_IP"

roslaunch decision limo_bringup.launch
