#!/usr/bin/env python3
"""
crosswalk_control.py
- /decision/mission == "CROSSWALK" 일 때만 동작
- 3초 정지 후 /decision/mission_done: "CROSSWALK_DONE" 퍼블리시
- 이후 차선 추종은 lane_control_autorace.py가 담당
"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

g_mission       = ""
g_stop_start    = None   # 정지 시작 시각
g_done_pub      = False  # 이미 done 퍼블리시 했는지

g_pub_cmd       = None
g_pub_done      = None
g_p             = {}


def load_params():
    global g_p
    p = rospy.get_param("~", {})
    g_p["stop_duration"] = float(p.get("stop_duration", 3.0))   # 정지 시간 [s]
    g_p["control_rate"]  = float(p.get("control_rate",  20.0))
    g_p["debug_mode"]    = bool(p.get("debug_mode",     False))
    rospy.loginfo("[crosswalk] stop_duration=%.1fs", g_p["stop_duration"])


def cb_mission(msg: String):
    global g_mission, g_stop_start, g_done_pub
    prev = g_mission
    g_mission = msg.data.strip()

    if g_mission == "CROSSWALK" and prev != "CROSSWALK":
        g_stop_start = rospy.Time.now()
        g_done_pub   = False
        rospy.loginfo("[crosswalk] CROSSWALK start — stopping %.1fs", g_p["stop_duration"])


def control_loop(event):
    global g_done_pub

    if not g_p["debug_mode"] and g_mission != "CROSSWALK":
        return

    # 정지 명령 발행
    g_pub_cmd.publish(Twist())

    if g_done_pub or g_stop_start is None:
        return

    elapsed = (rospy.Time.now() - g_stop_start).to_sec()
    if elapsed >= g_p["stop_duration"]:
        g_pub_done.publish(String(data="CROSSWALK_DONE"))
        g_done_pub = True
        rospy.loginfo("[crosswalk] stop complete → CROSSWALK_DONE")


def main():
    global g_pub_cmd, g_pub_done

    rospy.init_node("crosswalk_control")
    load_params()

    g_pub_cmd  = rospy.Publisher("/cmd_vel",              Twist,  queue_size=1)
    g_pub_done = rospy.Publisher("/decision/mission_done", String, queue_size=1)

    rospy.Subscriber("/decision/mission", String, cb_mission, queue_size=1)

    rospy.Timer(rospy.Duration(1.0 / g_p["control_rate"]), control_loop)

    rospy.loginfo("[crosswalk] node ready")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
