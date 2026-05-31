#!/usr/bin/env python3
"""
crosswalk_control.py (fixed)
/decision/mission == "CROSSWALK" 일 때:
  yellow_pixel_count >= yellow_thresh → 정지 시작
  stop_duration 초 후 → CROSSWALK_DONE 퍼블리시

수정사항:
  - 횡단보도 접근 시 저속 전진 (yellow 미감지 상태에서 정지하면 영원히 안 감)
  - shutdown hook 추가
"""

import rospy
from std_msgs.msg import String, Int32, Float32, Bool
from geometry_msgs.msg import Twist

# ── 전역 상태 ────────────────────────────────────────────────────
g_mission      = ""
g_yellow_count = 0
g_stop_start   = None
g_done         = False

g_pub_cmd      = None
g_pub_done     = None
g_p            = {}

# perception 값 (저속 전진용 간이 조향)
g_lateral  = 0.0
g_heading  = 0.0
g_detected = False


# ── 파라미터 로드 ────────────────────────────────────────────────
def load_params():
    global g_p
    p   = rospy.get_param("~", {})
    g_p = {
        "yellow_thresh":  int(p.get("yellow_thresh",   800)),
        "stop_duration":  float(p.get("stop_duration", 3.0)),
        "approach_speed": float(p.get("approach_speed", 0.12)),
        "control_rate":   float(p.get("control_rate",  20.0)),
    }
    rospy.loginfo("[crosswalk] yellow_thresh=%d  stop=%.1fs  approach_spd=%.2f",
                  g_p["yellow_thresh"], g_p["stop_duration"], g_p["approach_speed"])


# ── 콜백 ────────────────────────────────────────────────────────
def cb_mission(msg):
    global g_mission, g_stop_start, g_done
    prev      = g_mission
    g_mission = msg.data.strip()

    if g_mission == "CROSSWALK" and prev != "CROSSWALK":
        g_stop_start = None
        g_done       = False
        rospy.loginfo("[crosswalk] 시작 — yellow pixel 대기")


def cb_yellow(msg):
    global g_yellow_count
    g_yellow_count = msg.data


def cb_lateral(msg):
    global g_lateral
    g_lateral = msg.data

def cb_heading(msg):
    global g_heading
    g_heading = msg.data

def cb_detected(msg):
    global g_detected
    g_detected = msg.data


# ── 제어 루프 ────────────────────────────────────────────────────
def control_loop(event):
    global g_stop_start, g_done

    if g_mission != "CROSSWALK" or g_done:
        return

    cmd = Twist()

    # 아직 yellow 미감지 → 저속 전진하면서 횡단보도 접근
    if g_stop_start is None:
        if g_yellow_count >= g_p["yellow_thresh"]:
            g_stop_start = rospy.Time.now()
            rospy.loginfo("[crosswalk] yellow %d >= %d -> 정지",
                          g_yellow_count, g_p["yellow_thresh"])
            # 정지 cmd
            g_pub_cmd.publish(Twist())
            return

        # 저속 전진 + 간이 조향 (차선 이탈 방지)
        if g_detected:
            cmd.linear.x  = g_p["approach_speed"]
            cmd.angular.z = -0.5 * g_heading  # 간이 heading 보정
        else:
            cmd.linear.x = g_p["approach_speed"]
        g_pub_cmd.publish(cmd)
        return

    # yellow 감지 → 정지 유지
    g_pub_cmd.publish(Twist())

    if (rospy.Time.now() - g_stop_start).to_sec() >= g_p["stop_duration"]:
        g_pub_done.publish(String(data="CROSSWALK_DONE"))
        g_done = True
        rospy.loginfo("[crosswalk] 정지 완료 -> CROSSWALK_DONE")


# ── shutdown ─────────────────────────────────────────────────────
def on_shutdown():
    rospy.loginfo("[crosswalk] shutdown — zero cmd_vel")
    if g_pub_cmd is not None:
        g_pub_cmd.publish(Twist())


# ── main ─────────────────────────────────────────────────────────
def main():
    global g_pub_cmd, g_pub_done

    rospy.init_node("crosswalk_control")
    load_params()

    g_pub_cmd  = rospy.Publisher("/cmd_vel",               Twist,  queue_size=1)
    g_pub_done = rospy.Publisher("/decision/mission_done", String, queue_size=1)

    rospy.Subscriber("/decision/mission",              String, cb_mission,  queue_size=1)
    rospy.Subscriber("/perception/yellow_pixel_count", Int32,  cb_yellow,   queue_size=1)
    rospy.Subscriber("/perception/lateral_offset",     Float32, cb_lateral, queue_size=1)
    rospy.Subscriber("/perception/heading_error",      Float32, cb_heading, queue_size=1)
    rospy.Subscriber("/perception/lane_detected",      Bool,    cb_detected, queue_size=1)

    rospy.on_shutdown(on_shutdown)
    rospy.Timer(rospy.Duration(1.0 / g_p["control_rate"]), control_loop)

    rospy.loginfo("[crosswalk] node ready")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass