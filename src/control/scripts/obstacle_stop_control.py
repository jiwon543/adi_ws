#!/usr/bin/env python3
"""
obstacle_stop_control.py
/decision/mission == "EMERGENCY_STOP" 일 때:
  전방 장애물 감지 → 정지 유지
  장애물 사라지고 clear_sec 초 경과 → EMERGENCY_STOP_DONE 퍼블리시

구독: /decision/mission, /lidar/clusters
퍼블리시: /cmd_vel, /decision/mission_done
"""

import rospy
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# ── 전역 상태 ──────────────────────────────────────────────────────
g_mission   = ""
g_clusters  = []
g_done      = False

g_pub_cmd   = None
g_pub_done  = None
g_p         = {}

_clear_since = None


# ── 파라미터 로드 ────────────────────────────────────────────────
def load_params():
    global g_p
    p   = rospy.get_param("~", {})
    g_p = {
        "obstacle_dist":    float(p.get("obstacle_dist",    0.8)),
        "obstacle_y_thresh":float(p.get("obstacle_y_thresh", 0.35)),
        "clear_sec":        float(p.get("clear_sec",         1.0)),
        "control_rate":     float(p.get("control_rate",     20.0)),
    }
    rospy.loginfo("[obstacle_stop] params: %s", g_p)


# ── 장애물 전방 여부 ─────────────────────────────────────────────
def obstacle_in_front():
    d   = g_p["obstacle_dist"]
    lat = g_p["obstacle_y_thresh"]
    return any(0.05 < x < d and abs(y) < lat for x, y in g_clusters)


# ── 콜백 ────────────────────────────────────────────────────────
def cb_mission(msg):
    global g_mission, g_done, _clear_since
    prev      = g_mission
    g_mission = msg.data.strip()
    if g_mission == "EMERGENCY_STOP" and prev != "EMERGENCY_STOP":
        g_done       = False
        _clear_since = None
        rospy.loginfo("[obstacle_stop] 미션 활성화")


def cb_clusters(msg):
    global g_clusters
    g_clusters = [(pt.x, pt.y) for pt in msg.points]


# ── 제어 루프 ────────────────────────────────────────────────────
def control_loop(event):
    global g_done, _clear_since

    if g_mission != "EMERGENCY_STOP" or g_done:
        _clear_since = None
        return

    # 장애물 있는 동안 정지 유지
    g_pub_cmd.publish(Twist())

    if obstacle_in_front():
        _clear_since = None
    else:
        now = rospy.Time.now()
        if _clear_since is None:
            _clear_since = now
        elif (now - _clear_since).to_sec() >= g_p["clear_sec"]:
            rospy.loginfo("[obstacle_stop] 장애물 해소 → EMERGENCY_STOP_DONE")
            g_pub_done.publish(String(data="EMERGENCY_STOP_DONE"))
            g_done       = True
            _clear_since = None


# ── main ─────────────────────────────────────────────────────────
def main():
    global g_pub_cmd, g_pub_done

    rospy.init_node("obstacle_stop_control")
    load_params()

    g_pub_cmd  = rospy.Publisher("/cmd_vel",               Twist,  queue_size=1)
    g_pub_done = rospy.Publisher("/decision/mission_done", String, queue_size=1)

    rospy.Subscriber("/decision/mission", String,     cb_mission,  queue_size=1)
    rospy.Subscriber("/lidar/clusters",   PointCloud, cb_clusters, queue_size=1)

    rospy.Timer(rospy.Duration(1.0 / g_p["control_rate"]), control_loop)

    rospy.loginfo("[obstacle_stop] node ready")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
