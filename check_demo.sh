#!/bin/bash
# check_demo.sh — 송도 데모 프리플라이트 자동 진단
# 사용법:
#   리모에서:   ./check_demo.sh limo
#   노트북에서: ./check_demo.sh laptop
# 모든 노드(run_limo.sh + run_vlm.sh)를 띄운 뒤, 주행 시작 전에 실행할 것.

ROLE="${1:-limo}"
SETUP="/opt/ros/noetic/setup.bash"
[ -f "$SETUP" ] && source "$SETUP"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -f "$SCRIPT_DIR/devel/setup.bash" ] && source "$SCRIPT_DIR/devel/setup.bash"

OK="\033[32m[ OK ]\033[0m"
NO="\033[31m[FAIL]\033[0m"
WARN="\033[33m[WARN]\033[0m"

pass=0; fail=0
chk() { # chk "설명" "조건(0=성공)"
  if [ "$2" -eq 0 ]; then echo -e "$OK $1"; pass=$((pass+1));
  else echo -e "$NO $1"; fail=$((fail+1)); fi
}

echo "=================================================="
echo " 데모 프리플라이트 진단 — role=$ROLE"
echo "=================================================="

# ── 1. ROS 환경변수 ───────────────────────────────────
echo ""
echo "── 1) ROS 네트워크 환경 ──"
echo "  ROS_MASTER_URI = ${ROS_MASTER_URI:-(미설정)}"
echo "  ROS_IP         = ${ROS_IP:-(미설정)}"
echo "  ROS_HOSTNAME   = ${ROS_HOSTNAME:-(미설정)}"
[ -n "$ROS_MASTER_URI" ]; chk "ROS_MASTER_URI 설정됨" $?
# ROS_IP 가 IP 형태인지(권장) — hostname 쓰면 멀티머신서 자주 깨짐
if [[ "$ROS_IP" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  chk "ROS_IP 가 IP 주소 형태" 0
else
  echo -e "$WARN ROS_IP 가 IP 형태가 아님('${ROS_IP}') — 멀티머신서 hostname 은 자주 실패. export ROS_IP=<이 머신 IP> 권장"
fi
# 마스터 도달성
timeout 5 rostopic list >/dev/null 2>&1; chk "ROS master 도달 가능 (rostopic list)" $?
if [ "$ROLE" = "laptop" ]; then
  MIP=$(echo "$ROS_MASTER_URI" | sed -E 's#https?://([^:/]+).*#\1#')
  ping -c1 -W2 "$MIP" >/dev/null 2>&1; chk "마스터($MIP) ping 응답" $?
fi

have_topic() { rostopic list 2>/dev/null | grep -qx "$1"; }
hz_of() { timeout 6 rostopic hz "$1" 2>/dev/null | grep -m1 "average rate" | grep -oE "[0-9]+\.[0-9]+" | head -1; }

# ── 2. 센서/인지 토픽 (리모에서 발행) ─────────────────
echo ""
echo "── 2) 센서·인지 토픽 발행 여부 ──"
have_topic "/camera/color/image_raw/compressed"; chk "카메라 compressed 토픽 존재 (★VLM·차선 둘 다 필수)" $?
have_topic "/scan";                              chk "라이다 /scan 토픽 존재" $?
have_topic "/perception/lateral_offset";         chk "차선 인지 /perception/lateral_offset 발행" $?
have_topic "/perception/yellow_pixel_count";     chk "횡단보도 노란픽셀 토픽 발행" $?
have_topic "/lidar/clusters";                    chk "라이다 클러스터 /lidar/clusters 발행" $?

# 실시간 주파수(0 이면 노드는 떴는데 데이터가 안 흐르는 것)
echo ""
echo "  [실시간 Hz — 0 이면 데이터 흐름 끊김]"
for t in /camera/color/image_raw/compressed /scan /perception/lateral_offset /lidar/clusters /vlm/mission /decision/mission /cmd_vel; do
  printf "    %-42s %s Hz\n" "$t" "$(hz_of $t || echo 0)"
done

# ── 3. 의사결정/VLM 연결 ──────────────────────────────
echo ""
echo "── 3) 의사결정·VLM 파이프라인 ──"
have_topic "/vlm/mission";       chk "VLM /vlm/mission 발행 (★노트북 VLM 연결 증거)" $?
have_topic "/decision/mission";  chk "decision /decision/mission 발행" $?
echo "  최근 decision/mission 상태:"
timeout 3 rostopic echo -n1 /decision/mission 2>/dev/null | sed 's/^/    /' || echo "    (수신 없음)"
echo "  최근 VLM 판단(candidate / raw):"
timeout 3 rostopic echo -n1 /vlm/mission 2>/dev/null | sed 's/^/    /' || echo "    (수신 없음 — R1/R2 확인)"

# ── 4. 액추에이터 ─────────────────────────────────────
echo ""
echo "── 4) 구동 ──"
have_topic "/cmd_vel"; chk "/cmd_vel 토픽 존재 (LIMO base 구독 대상)" $?

# ── 5. 임계값 라이브 튜닝 도우미 ──────────────────────
echo ""
echo "── 5) 임계값 실측 (콘/횡단보도 깔고 차를 그 앞에 둔 상태에서 실행) ──"
echo "  ▶ 콘 검증용 — 1.5m·±0.7m 안 클러스터 수가 cone_min_clusters(현재 4) 이상이어야 회피 발동:"
echo "     rostopic echo /lidar/clusters | grep -c 'x:'   (한 메시지의 점 개수 확인)"
echo "  ▶ 횡단보도 — 차가 횡단보도 위일 때 값이 yellow_thresh(현재 4000) 이상이어야 정지:"
LATEST_YELLOW=$(timeout 3 rostopic echo -n1 /perception/yellow_pixel_count 2>/dev/null | grep -oE '[0-9]+' | head -1)
echo "     현재 yellow_pixel_count = ${LATEST_YELLOW:-(수신없음)}  (임계 4000)"

echo ""
echo "=================================================="
echo -e " 결과:  통과 $pass  /  실패 $fail"
if [ "$fail" -eq 0 ]; then
  echo -e " $OK 핵심 토픽 정상 — 주행 시작 가능"
else
  echo -e " $NO 실패 항목 있음 — DEMO_CHECKLIST.md 의 해당 R# 참고"
fi
echo "=================================================="
