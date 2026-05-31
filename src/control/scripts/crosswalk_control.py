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
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# ── 전역 상태 ────────────────────────────────────────────────────
g_mission    = ""
g_stop_start = None
g_done       = False

g_pub_cmd    = None
g_pub_done   = None
g_p          = {}


# ── 파라미터 로드 ────────────────────────────────────────────────
def load_params():
    global g_p
    p   = rospy.get_param("~", {})
    g_p = {
        "stop_duration":  float(p.get("stop_duration", 3.0)),
        "control_rate":   float(p.get("control_rate",  20.0)),
    }
    rospy.loginfo("[crosswalk] stop=%.1fs", g_p["stop_duration"])


# ── 콜백 ────────────────────────────────────────────────────────
def cb_mission(msg):
    global g_mission, g_stop_start, g_done
    prev      = g_mission
    g_mission = msg.data.strip()

    if g_mission == "CROSSWALK" and prev != "CROSSWALK":
        g_done       = False
        g_stop_start = rospy.Time.now()   # decision 이 이미 검증 완료 → 즉시 정지
        rospy.loginfo("[crosswalk] CROSSWALK 진입 — 즉시 정지 시작")




# ── 제어 루프 ────────────────────────────────────────────────────
def control_loop(event):
    global g_stop_start, g_done

    if g_mission != "CROSSWALK" or g_done:
        return

    # 정지 유지 (stop_start 는 cb_mission 에서 즉시 설정됨)
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

    rospy.Subscriber("/decision/mission", String, cb_mission, queue_size=1)

    rospy.on_shutdown(on_shutdown)
    rospy.Timer(rospy.Duration(1.0 / g_p["control_rate"]), control_loop)

    rospy.loginfo("[crosswalk] node ready")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass