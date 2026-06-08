#!/usr/bin/env python3
"""
demo_viz_node_v2.py — Decision 디버깅 시각화 노드 v2

Layout (1280x720):
  [ BANNER: decision/mission (좌+중간 920px) │ Debug panel (360px, 풀높이) ]
  [ Camera (360)  │ BEV (560, 75%)           │                             ]
  [               │ LiDAR map (560, 25%)      │                             ]

Subscribes:
  /camera/color/image_raw/compressed    (CompressedImage)
  /perception/bev                       (Image)
  /decision/mission                     (String)
  /vlm/mission                          (String, JSON)
  /lidar/clusters                       (PointCloud)
  /perception/yellow_pixel_count        (Int32)

Publishes:
  /demo/viz_v2   (Image)
"""

import json
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image, PointCloud
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge

# ── 미션별 색상 (BGR) ─────────────────────────────────────────────
MISSION_COLOR = {
    "LANE_FOLLOW":    (50,  200,  80),
    "CONE_AVOID":     (30,  140, 255),
    "CROSSWALK":      (0,   210, 220),
    "EMERGENCY_STOP": (40,   60, 240),   # obstacle stop
    "WAIT":           (120, 120, 120),
}
MISSION_LABEL = {
    "LANE_FOLLOW":    "LANE FOLLOW",
    "CONE_AVOID":     "CONE AVOID",
    "CROSSWALK":      "CROSSWALK STOP",
    "EMERGENCY_STOP": "OBSTACLE STOP",
    "WAIT":           "WAITING (no VLM)",
}
MISSION_DESC = {
    "LANE_FOLLOW":    "Stanley lane following",
    "CONE_AVOID":     "Cone avoidance active",
    "CROSSWALK":      "Crosswalk — 3s stop",
    "EMERGENCY_STOP": "Obstacle stop",
    "WAIT":           "VLM not connected",
}

_bridge = CvBridge()

# ── 전역 상태 ──────────────────────────────────────────────────────
g_camera_bgr    = None
g_bev_bgr       = None
g_mission       = "LANE_FOLLOW"
g_vlm_candidate = "normal_drive"
g_vlm_raw       = ""
g_vlm_time      = None
g_clusters      = []
g_yellow_count  = 0
g_start_time    = None

# CROSSWALK 완료 후 2초간 표시 래치
g_crosswalk_latch_until = None

g_pub_viz      = None
g_video_writer = None
g_p            = {}

YELLOW_THRESH    = 4000   # decision_node.yaml 과 동기화
CROSSWALK_LATCH  = 3.0    # CROSSWALK 완료 후 표시 유지 [s] (decision crosswalk_cooldown: 10s)


# ── 콜백 ──────────────────────────────────────────────────────────
def cb_camera(msg):
    global g_camera_bgr, g_start_time
    arr = np.frombuffer(msg.data, np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is not None:
        g_camera_bgr = bgr
    if g_start_time is None:
        g_start_time = rospy.Time.now()


def cb_bev(msg):
    global g_bev_bgr
    try:
        g_bev_bgr = _bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception:
        pass


def cb_mission(msg):
    global g_mission, g_crosswalk_latch_until
    prev      = g_mission
    g_mission = msg.data.strip()
    if prev == "CROSSWALK" and g_mission != "CROSSWALK":
        g_crosswalk_latch_until = rospy.Time.now() + rospy.Duration(CROSSWALK_LATCH)


def cb_vlm(msg):
    global g_vlm_candidate, g_vlm_raw, g_vlm_time
    try:
        j = json.loads(msg.data)
        g_vlm_candidate = j.get("candidate", "normal_drive")
        g_vlm_raw       = j.get("vlm_raw", "")[:70]
        g_vlm_time      = rospy.Time.now()
    except Exception:
        pass


def cb_clusters(msg):
    global g_clusters
    g_clusters = [(pt.x, pt.y) for pt in msg.points]


def cb_yellow(msg):
    global g_yellow_count
    g_yellow_count = msg.data


# ── 유틸 ──────────────────────────────────────────────────────────
def puttext(img, text, pos, scale=0.6, color=(255, 255, 255), thick=1):
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX,
                scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, thick, cv2.LINE_AA)


def effective_mission(now):
    """래치 포함 표시용 미션 반환"""
    if g_crosswalk_latch_until and now < g_crosswalk_latch_until:
        return "CROSSWALK"
    return g_mission


# ── 상단 배너 (좌+중간 컬럼만) ───────────────────────────────────
def draw_top_banner(canvas, banner_w, BH):
    now   = rospy.Time.now()
    state = effective_mission(now)
    mc     = MISSION_COLOR.get(state, (120, 120, 120))
    lbl    = MISSION_LABEL.get(state, state)

    cv2.rectangle(canvas, (0, 0), (banner_w, BH), tuple(max(0, c // 5) for c in mc), -1)
    cv2.rectangle(canvas, (0, BH - 4), (banner_w, BH), mc, -1)

    puttext(canvas, "decision/mission:", (10, BH - 28), 0.52, (140, 140, 140), 1)

    ts = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 1.1, 2)[0]
    puttext(canvas, lbl, ((banner_w - ts[0]) // 2, BH - 8), 1.1, mc, 2)


# ── LiDAR 미니맵 ──────────────────────────────────────────────────
def draw_lidar_map(clusters, w, h):
    mm  = np.full((h, w, 3), (18, 18, 28), dtype=np.uint8)
    cx  = w // 2
    cy  = h - 16
    sx  = (h - 36) / 5.0    # 전방 5m
    sy  = (w // 2 - 10) / 2.0

    for (x, yr) in clusters:
        if x <= 0.0 or x > 5.0:
            continue
        px = cx - int(yr * sy)
        py = cy - int(x  * sx)
        if 0 <= px < w and 0 <= py < h:
            color = (40, 60, 240) if np.hypot(x, yr) < 0.8 else (40, 180, 80)
            cv2.circle(mm, (px, py), 5, color, -1)

    cv2.circle(mm, (cx, cy), 7, (0, 220, 255), -1)
    puttext(mm, "LiDAR  (DBSCAN)", (6, 16), 0.45, (160, 160, 200))
    return mm


# ── 우측 디버그 패널 ──────────────────────────────────────────────
def build_debug_panel(pw, ph):
    panel = np.full((ph, pw, 3), (14, 14, 22), dtype=np.uint8)
    now   = rospy.Time.now()
    y     = 6

    def sec_header(label, color):
        nonlocal y
        cv2.rectangle(panel, (0, y), (pw, y + 22), tuple(c // 6 for c in color), -1)
        puttext(panel, label, (6, y + 15), 0.52, color, 1)
        y += 26

    # ── Decision FSM ──
    sec_header("Decision FSM", (140, 190, 255))

    state = effective_mission(now)
    mc    = MISSION_COLOR.get(state, (180, 180, 180))
    lbl   = MISSION_LABEL.get(state, state)
    desc  = MISSION_DESC.get(state, "")
    puttext(panel, f"state : {lbl}", (8, y + 14), 0.5, mc)
    y += 20
    puttext(panel, f"  {desc}", (8, y + 13), 0.44, tuple(min(255, c + 60) for c in mc))
    y += 22

    # ── VLM ──
    sec_header("VLM  (Moondream2)", (150, 230, 150))

    vlm_age   = (now - g_vlm_time).to_sec() if g_vlm_time else 99.0
    age_color = (80, 220, 100) if vlm_age < 1.0 else \
                (200, 160,  40) if vlm_age < 3.0 else (100, 100, 100)
    puttext(panel, f"candidate : {g_vlm_candidate}", (8, y + 14), 0.5, age_color)
    y += 19
    puttext(panel, f"hint age  : {vlm_age:.2f}s", (8, y + 13), 0.46, age_color)
    y += 17

    raw = g_vlm_raw
    for i in range(0, min(len(raw), 70), 36):
        puttext(panel, f'"{raw[i:i+36]}"', (8, y + 13), 0.41, (165, 165, 165))
        y += 15
    y += 8

    # ── LiDAR 도트맵 ──
    sec_header("LiDAR  (DBSCAN)", (200, 150, 255))

    map_w = pw - 16
    map_h = ph - y - 8
    if map_h > 60:
        mm = draw_lidar_map(g_clusters, map_w, map_h)
        panel[y: y + map_h, 8: 8 + map_w] = mm

    return panel


# ── 이미지 서브윈도우 붙여넣기 ────────────────────────────────────
def paste_fit(canvas, img, x0, y0, w, h, label=None):
    if img is None:
        cv2.rectangle(canvas, (x0, y0), (x0 + w, y0 + h), (28, 28, 38), -1)
        puttext(canvas, "no signal", (x0 + w // 2 - 45, y0 + h // 2), 0.55, (75, 75, 75))
        if label:
            puttext(canvas, label, (x0 + 6, y0 + 20), 0.52, (90, 90, 90))
        return
    ih, iw = img.shape[:2]
    scale  = min(w / iw, h / ih)
    nw, nh = int(iw * scale), int(ih * scale)
    canvas[y0 + (h - nh) // 2: y0 + (h - nh) // 2 + nh,
           x0 + (w - nw) // 2: x0 + (w - nw) // 2 + nw] = cv2.resize(img, (nw, nh))
    if label:
        lw = len(label) * 9 + 12
        cv2.rectangle(canvas, (x0, y0), (x0 + lw, y0 + 24), (0, 0, 0), -1)
        puttext(canvas, label, (x0 + 6, y0 + 17), 0.52, (200, 200, 200))


# ── 프레임 합성 ───────────────────────────────────────────────────
def render_frame():
    W, H     = g_p["width"], g_p["height"]
    BANNER_H = 60

    DBG_W    = 360
    BANNER_W = W - DBG_W             # 920px (배너 = 좌+중간)
    CAM_W    = int(BANNER_W * 0.6)   # 552px (60%)
    MID_W    = BANNER_W - CAM_W      # 368px (40%)
    MID_H    = H - BANNER_H

    frame = np.zeros((H, W, 3), dtype=np.uint8)

    # 우: 디버그 패널 (풀 높이, 배너 없음)
    frame[0:H, BANNER_W:W] = build_debug_panel(DBG_W, H)

    # 배너 (좌+중간만)
    if g_start_time:
        draw_top_banner(frame, BANNER_W, BANNER_H)

    Y0 = BANNER_H

    # 좌: 카메라 (60%)
    paste_fit(frame, g_camera_bgr, 0, Y0, CAM_W, MID_H, label="Camera")

    # 중: BEV 풀 높이 (LiDAR는 우측 패널로 이동)
    paste_fit(frame, g_bev_bgr, CAM_W, Y0, MID_W, MID_H, label="BEV")

    # 구분선
    cv2.line(frame, (0, BANNER_H),     (BANNER_W, BANNER_H), (80, 80, 100), 1)
    cv2.line(frame, (CAM_W, BANNER_H), (CAM_W, H),           (55, 55, 75),  2)
    cv2.line(frame, (BANNER_W, 0),     (BANNER_W, H),         (55, 55, 75),  2)

    return frame


# ── Timer callback ────────────────────────────────────────────────
def timer_cb(event):
    if g_camera_bgr is None:
        return
    frame = render_frame()
    try:
        g_pub_viz.publish(_bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
    except Exception:
        pass
    if g_video_writer is not None:
        g_video_writer.write(frame)


def on_shutdown():
    if g_video_writer is not None:
        g_video_writer.release()
        rospy.loginfo("[demo_viz_v2] 저장 완료: %s", g_p["output_path"])


# ── main ──────────────────────────────────────────────────────────
def main():
    global g_pub_viz, g_video_writer, g_p

    rospy.init_node("demo_viz_node_v2")
    g_p = {
        "save_video":  bool(rospy.get_param("~save_video",   False)),
        "output_path": str(rospy.get_param("~output_path",   "/tmp/demo_output_v2.mp4")),
        "fps":         float(rospy.get_param("~fps",         20.0)),
        "width":       int(rospy.get_param("~width",         1280)),
        "height":      int(rospy.get_param("~height",        720)),
    }

    g_pub_viz = rospy.Publisher("/demo/viz_v2", Image, queue_size=1)

    if g_p["save_video"]:
        fourcc         = cv2.VideoWriter_fourcc(*"mp4v")
        g_video_writer = cv2.VideoWriter(
            g_p["output_path"], fourcc,
            g_p["fps"], (g_p["width"], g_p["height"])
        )
        rospy.loginfo("[demo_viz_v2] MP4 저장: %s", g_p["output_path"])

    rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage,
                     cb_camera,  queue_size=1, buff_size=2 ** 24)
    rospy.Subscriber("/perception/bev",                    Image,
                     cb_bev,     queue_size=1, buff_size=2 ** 24)
    rospy.Subscriber("/decision/mission",                  String,     cb_mission,  queue_size=1)
    rospy.Subscriber("/vlm/mission",                       String,     cb_vlm,      queue_size=1)
    rospy.Subscriber("/lidar/clusters",                    PointCloud, cb_clusters, queue_size=1)
    rospy.Subscriber("/perception/yellow_pixel_count",     Int32,      cb_yellow,   queue_size=1)

    rospy.on_shutdown(on_shutdown)
    rospy.Timer(rospy.Duration(1.0 / g_p["fps"]), timer_cb)

    rospy.loginfo("[demo_viz_v2] ready — rqt_image_view /demo/viz_v2")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
