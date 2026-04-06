#!/usr/bin/env python3
"""
cone_avoidance_control.py  (rev-2)

변경 사항 (cpp mission_labacorn 로직 반영):
  1. stop_dist 미사용 버그 수정 → 근접 콘 감지 시 2단계 회피 FSM 진입
  2. 회피 FSM: REVERSE(후진+최대조향) → STOP(제동+조향 유지) → NONE (cpp 동일)
  3. 회피 완료 후 PID 적분·미분항 초기화 추가
  4. 차선 소실 판단 로직 재작성 (원본 버그: detected=False이면 타임아웃 무관 즉시 정지)
  5. 원거리 콘: gap 중심 조향 유지 (기존 py 로직)
  6. 로그 통일 (throttle 레이어별 분리)

구독: /lidar/clusters, /perception/lateral_offset, /perception/lane_detected
퍼블리시: /cmd_vel
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist

# ── 회피 FSM 상태 ──────────────────────────────────────────────
AVOID_NONE    = 0   # 정상 주행
AVOID_REVERSE = 1   # 후진 + 최대 조향
AVOID_STOP    = 2   # 제동 + 조향 유지

# ── 전역 상태 ─────────────────────────────────────────────────
g_cones: list            = []
g_lateral: float         = 0.0
g_detected: bool         = False
g_last_detect_time       = None   # ros::Time | None

g_err_prev: float        = 0.0
g_err_sum:  float        = 0.0

g_avoid_state: int       = AVOID_NONE
g_avoid_left:  bool      = True   # True → 왼쪽 장애물 → 오른쪽으로 회피
g_avoid_start_time       = None   # ros::Time | None

g_pub_cmd                = None
g_p: dict                = {}


# ── 파라미터 로드 ──────────────────────────────────────────────
def load_params() -> None:
    global g_p
    p = rospy.get_param("~", {})

    # 콘 감지 범위
    g_p["avoid_dist"]      = float(p.get("avoid_dist",      1.2))   # 전방 감지 한계 [m]
    g_p["lateral_limit"]   = float(p.get("lateral_limit",   0.6))   # 좌우 감지 폭 [m]
    g_p["stop_dist"]       = float(p.get("stop_dist",       0.35))  # 이 거리 미만 → 강제 회피 FSM 진입

    # 회피 FSM (cpp avoid_hold_sec / avoid_stop_sec 대응)
    g_p["avoid_hold_sec"]  = float(p.get("avoid_hold_sec",  0.5))   # REVERSE 유지 시간 [s]
    g_p["avoid_stop_sec"]  = float(p.get("avoid_stop_sec",  1.0))   # STOP 유지 시간 [s]
    g_p["reverse_speed"]   = float(p.get("reverse_speed",  -0.2))   # 후진 속도 [m/s]

    # 원거리 콘 gap 조향
    g_p["avoid_gain"]      = float(p.get("avoid_gain",      1.5))   # gap 반발 게인
    g_p["avoid_speed"]     = float(p.get("avoid_speed",     0.2))   # 원거리 회피 전진 속도

    # 최대 조향각 (AVOID_REVERSE/STOP에서도 사용)
    g_p["max_steering"]    = float(p.get("max_steering",    0.5))

    # 차선 PID
    g_p["kp"]              = float(p.get("kp",              0.01))
    g_p["ki"]              = float(p.get("ki",              0.0))
    g_p["kd"]              = float(p.get("kd",              0.005))
    g_p["i_max"]           = float(p.get("i_max",           50.0))

    # 정상 주행 속도 / 차선 소실 타임아웃
    g_p["base_speed"]      = float(p.get("base_speed",      0.3))
    g_p["lost_timeout"]    = float(p.get("lost_timeout",    1.0))
    g_p["control_rate"]    = float(p.get("control_rate",    30.0))

    rospy.loginfo("[cone_ctrl] params loaded: %s", g_p)


# ── 콘 필터링 ─────────────────────────────────────────────────
def filter_cones(cloud_points) -> list:
    """
    danger zone 내 클러스터만 추출
    laser_link 기준: x=전방(+), y=좌(+)/우(-)
    """
    result = []
    for pt in cloud_points:
        x, y = pt.x, pt.y
        if 0.05 < x < g_p["avoid_dist"] and abs(y) < g_p["lateral_limit"]:
            result.append((x, y))
    return result


def compute_gap_steering(cones: list) -> float:
    left_cones  = [(x, y) for x, y in cones if y >= 0]
    right_cones = [(x, y) for x, y in cones if y <  0]

    if left_cones and right_cones:
        inner_left_y  = min(y for _, y in left_cones)
        inner_right_y = max(y for _, y in right_cones)
        gap_y = (inner_left_y + inner_right_y) / 2.0
        # gap이 왼쪽(양수) → 왼쪽으로 가야 함 → angular.z 양수
        steering = g_p["avoid_gain"] * gap_y          # 부호 수정: - → +
    elif left_cones:
        nearest_y = min(y for _, y in left_cones)     # 양수
        steering  = -g_p["avoid_gain"] * nearest_y    # 음수 → 오른쪽 ✓
    else:
        nearest_y = max(y for _, y in right_cones)    # 음수
        steering  = -g_p["avoid_gain"] * nearest_y    # 양수 → 왼쪽 ✓

    return float(np.clip(steering, -g_p["max_steering"], g_p["max_steering"]))

# ── 콜백 ──────────────────────────────────────────────────────
def cb_clusters(msg) -> None:
    global g_cones
    g_cones = filter_cones(msg.points)


def cb_lateral(msg) -> None:
    global g_lateral
    g_lateral = msg.data


def cb_detected(msg) -> None:
    global g_detected, g_last_detect_time
    g_detected = msg.data
    if g_detected:
        g_last_detect_time = rospy.Time.now()


# ── 제어 루프 ─────────────────────────────────────────────────
def control_loop(event) -> None:
    global g_err_prev, g_err_sum
    global g_avoid_state, g_avoid_left, g_avoid_start_time

    now   = rospy.Time.now()
    cmd   = Twist()
    cones = list(g_cones)  # 콜백 레이스 방지용 로컬 복사

    # ─────────────────────────────────────────────────────────
    # 1) 회피 FSM 트리거 (AVOID_NONE 상태에서만 체크)
    # ─────────────────────────────────────────────────────────
    if g_avoid_state == AVOID_NONE:
        close_left  = [c for c in cones if c[0] < g_p["stop_dist"] and c[1] >= 0]
        close_right = [c for c in cones if c[0] < g_p["stop_dist"] and c[1] <  0]

        if close_left or close_right:
            # 왼쪽 장애물 우선 (cpp: left_close || right_close, left 우선)
            g_avoid_left      = bool(close_left)
            g_avoid_state     = AVOID_REVERSE
            g_avoid_start_time = now
            rospy.logwarn(
                "[cone_ctrl] AVOID triggered — close_L=%d, close_R=%d, dir=%s",
                len(close_left), len(close_right),
                "LEFT" if g_avoid_left else "RIGHT"
            )

    # ─────────────────────────────────────────────────────────
    # 2) AVOID_REVERSE: 후진 + 최대 조향 (avoid_hold_sec 동안)
    # ─────────────────────────────────────────────────────────
    if g_avoid_state == AVOID_REVERSE:
        elapsed = (now - g_avoid_start_time).to_sec()

        if elapsed < g_p["avoid_hold_sec"]:
            cmd.linear.x  = g_p["reverse_speed"]
            # 왼쪽 장애물 → 오른쪽으로 최대 조향 (angular.z 음수)
            # 오른쪽 장애물 → 왼쪽으로 최대 조향 (angular.z 양수)
            cmd.angular.z = -g_p["max_steering"] if g_avoid_left else g_p["max_steering"]
            rospy.loginfo_throttle(
                0.2, "[cone_ctrl][REVERSE] elapsed=%.2f  steer=%.2f  spd=%.2f",
                elapsed, cmd.angular.z, cmd.linear.x
            )
        else:
            g_avoid_state      = AVOID_STOP
            g_avoid_start_time = now
            rospy.loginfo("[cone_ctrl] REVERSE done → STOP")

        g_pub_cmd.publish(cmd)
        return

    # ─────────────────────────────────────────────────────────
    # 3) AVOID_STOP: 제동 + 조향 유지 (avoid_stop_sec 동안)
    # ─────────────────────────────────────────────────────────
    if g_avoid_state == AVOID_STOP:
        elapsed = (now - g_avoid_start_time).to_sec()

        if elapsed < g_p["avoid_stop_sec"]:
            cmd.linear.x  = 0.0
            cmd.angular.z = -g_p["max_steering"] if g_avoid_left else g_p["max_steering"]
            rospy.loginfo_throttle(
                0.2, "[cone_ctrl][STOP] elapsed=%.2f  steer=%.2f",
                elapsed, cmd.angular.z
            )
        else:
            # 회피 완료 → PID 상태 초기화 후 NONE 복귀
            g_avoid_state = AVOID_NONE
            g_err_prev    = 0.0
            g_err_sum     = 0.0
            rospy.loginfo("[cone_ctrl] AVOID complete → LANE_FOLLOW")

        g_pub_cmd.publish(cmd)
        return

    # ─────────────────────────────────────────────────────────
    # 4) 원거리 콘: gap 중심 조향으로 저속 전진 (stop_dist 이상)
    # ─────────────────────────────────────────────────────────
    if cones:
        steering      = compute_gap_steering(cones)
        cmd.linear.x  = g_p["avoid_speed"]
        cmd.angular.z = steering
        g_pub_cmd.publish(cmd)
        rospy.loginfo_throttle(
            0.5, "[cone_ctrl][GAP] cones=%d  steer=%.3f  spd=%.2f",
            len(cones), steering, cmd.linear.x
        )
        return

    # ─────────────────────────────────────────────────────────
    # 5) 차선 추종 PID
    # 원본 버그 수정: detected=False 여도 lost_timeout 이내면 마지막 offset으로 주행
    # ─────────────────────────────────────────────────────────
    lane_lost = False
    if not g_detected:
        if g_last_detect_time is None:
            lane_lost = True
        elif (now - g_last_detect_time).to_sec() > g_p["lost_timeout"]:
            lane_lost = True

    if lane_lost:
        # 차선 완전 소실 → 정지
        g_pub_cmd.publish(cmd)
        rospy.logwarn_throttle(1.0, "[cone_ctrl] lane LOST — stopping")
        return

    error      = g_lateral
    derivative = error - g_err_prev
    g_err_sum  = float(np.clip(
        g_err_sum + error, -g_p["i_max"], g_p["i_max"]
    ))

    steering = float(np.clip(
        g_p["kp"] * error + g_p["ki"] * g_err_sum + g_p["kd"] * derivative,
        -g_p["max_steering"], g_p["max_steering"]
    ))
    g_err_prev = error

    cmd.linear.x  = g_p["base_speed"]
    cmd.angular.z = -steering   # lateral_offset 부호에 맞춰 반전

    g_pub_cmd.publish(cmd)
    rospy.loginfo_throttle(
        0.5, "[cone_ctrl][LANE] err=%.3f  steer=%.3f  spd=%.2f",
        error, steering, cmd.linear.x
    )


# ── main ──────────────────────────────────────────────────────
def main() -> None:
    global g_pub_cmd

    rospy.init_node("cone_avoidance_control")
    load_params()

    g_pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    rospy.Subscriber("/lidar/clusters",            PointCloud, cb_clusters, queue_size=1)
    rospy.Subscriber("/perception/lateral_offset", Float32,    cb_lateral,  queue_size=1)
    rospy.Subscriber("/perception/lane_detected",  Bool,       cb_detected, queue_size=1)

    rospy.Timer(rospy.Duration(1.0 / g_p["control_rate"]), control_loop)

    rospy.loginfo("[cone_ctrl] node started (rate=%.1f Hz)", g_p["control_rate"])
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass