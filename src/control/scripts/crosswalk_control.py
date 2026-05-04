#!/usr/bin/env python3
"""
crosswalk_control.py
/decision/mission == "CROSSWALK" 일 때:
  yellow_pixel_count >= yellow_thresh → 정지 시작
  stop_duration 초 후 → CROSSWALK_DONE 퍼블리시 → 차선 추종 재개
"""

import rospy
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist

# ── 전역 상태 ────────────────────────────────────────────────────
g_mission      = ""
g_yellow_count = 0
g_stop_start   = None   # 정지 시작 시각 (None = 대기 중)
g_done         = False  # CROSSWALK_DONE 퍼블리시 완료 여부

g_pub_cmd      = None
g_pub_done     = None
g_p            = {}


# ── 파라미터 로드 ────────────────────────────────────────────────
def load_params():
    global g_p
    p   = rospy.get_param("~", {})
    g_p = {
        "yellow_thresh": int(p.get("yellow_thresh",   800)),
        "stop_duration": float(p.get("stop_duration", 3.0)),
        "control_rate":  float(p.get("control_rate",  20.0)),
    }
    rospy.loginfo("[crosswalk] yellow_thresh=%d  stop=%.1fs",
                  g_p["yellow_thresh"], g_p["stop_duration"])


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


# ── 제어 루프 ────────────────────────────────────────────────────
def control_loop(event):
    global g_stop_start, g_done

    if g_mission != "CROSSWALK" or g_done:
        return

    if g_stop_start is None:
        if g_yellow_count >= g_p["yellow_thresh"]:
            g_stop_start = rospy.Time.now()
            rospy.loginfo("[crosswalk] yellow %d ≥ %d → 정지",
                          g_yellow_count, g_p["yellow_thresh"])
        return

    g_pub_cmd.publish(Twist())

    if (rospy.Time.now() - g_stop_start).to_sec() >= g_p["stop_duration"]:
        g_pub_done.publish(String(data="CROSSWALK_DONE"))
        g_done = True
        rospy.loginfo("[crosswalk] 정지 완료 → CROSSWALK_DONE")


# ── main ─────────────────────────────────────────────────────────
def main():
    global g_pub_cmd, g_pub_done

    rospy.init_node("crosswalk_control")
    load_params()

    g_pub_cmd  = rospy.Publisher("/cmd_vel",               Twist,  queue_size=1)
    g_pub_done = rospy.Publisher("/decision/mission_done", String, queue_size=1)

    rospy.Subscriber("/decision/mission",              String, cb_mission, queue_size=1)
    rospy.Subscriber("/perception/yellow_pixel_count", Int32,  cb_yellow,  queue_size=1)

    rospy.Timer(rospy.Duration(1.0 / g_p["control_rate"]), control_loop)

    rospy.loginfo("[crosswalk] node ready")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
