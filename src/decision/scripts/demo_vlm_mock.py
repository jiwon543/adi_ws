#!/usr/bin/env python3
"""
demo_vlm_mock.py — 모의 VLM 퍼블리셔

bag에 /vlm/mission 토픽이 없을 때 사용.
bag 첫 프레임을 기준으로 경과 시간에 따라 VLM 힌트를 주입한다.

Pub: /vlm/mission (String, JSON)

Params:
  ~publish_rate  (float, 2.0)   Hz
  ~schedule      (list)         vlm_mock.yaml 참고
"""

import json
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

g_pub       = None
g_bag_start = None
g_schedule  = []


def cb_camera(msg):
    global g_bag_start
    if g_bag_start is None:
        g_bag_start = rospy.Time.now()
        rospy.loginfo("[vlm_mock] bag 감지 — 타이머 시작 (t=0)")


def get_hint(t_elapsed):
    for item in g_schedule:
        if item["t_start"] <= t_elapsed < item["t_end"]:
            return item["candidate"], item["vlm_raw"]
    return "normal_drive", "clear road ahead"


def publish_cb(event):
    if g_bag_start is None:
        return

    t = (rospy.Time.now() - g_bag_start).to_sec()
    candidate, vlm_raw = get_hint(t)

    payload = json.dumps({
        "candidate":        candidate,
        "vlm_raw":          vlm_raw,
        "vlm_publish_time": rospy.Time.now().to_sec(),
    })
    g_pub.publish(String(data=payload))


def main():
    global g_pub, g_schedule

    rospy.init_node("demo_vlm_mock")
    rospy.logwarn("[vlm_mock] 모의 VLM 동작 중 (실제 VLM 대체)")

    raw_schedule = rospy.get_param("~schedule", [])
    for item in raw_schedule:
        g_schedule.append({
            "t_start":   float(item.get("t_start",   0.0)),
            "t_end":     float(item.get("t_end",  9999.0)),
            "candidate": str(item.get("candidate",  "normal_drive")),
            "vlm_raw":   str(item.get("vlm_raw",    "clear road")),
        })
    rospy.loginfo("[vlm_mock] %d 구간 로드됨: %s",
                  len(g_schedule),
                  [(s["t_start"], s["candidate"]) for s in g_schedule])

    rate = float(rospy.get_param("~publish_rate", 2.0))

    g_pub = rospy.Publisher("/vlm/mission", String, queue_size=1)
    rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage,
                     cb_camera, queue_size=1)

    rospy.Timer(rospy.Duration(1.0 / rate), publish_cb)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
