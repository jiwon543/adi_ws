#!/usr/bin/env python3
"""
demo_viz_node.py — Decision 디버깅 시각화 노드

Layout (1280x720):
  Left  : 원본 카메라 | BEV 차선 영상 (각각 절반)
  Right : Decision 패널 (VLM output / Decision FSM / LiDAR)

Subscribes:
  /camera/color/image_raw/compressed   (CompressedImage)
  /perception/bev                      (Image)
  /decision/mission                    (String)
  /vlm/mission                         (String, JSON)
  /lidar/clusters                      (PointCloud)

Publishes:
  /demo/viz   (Image)

Params:
  ~save_video   (bool,  false)
  ~output_path  (str,   /tmp/demo_output.mp4)
  ~fps          (float, 20.0)
  ~width        (int,   1280)
  ~height       (int,   720)
"""

import json
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image, PointCloud
from std_msgs.msg import String
from cv_bridge import CvBridge

# ── 미션별 색상 (BGR) ─────────────────────────────────────────────
MISSION_COLOR = {
    "LANE_FOLLOW":    (50,  200,  80),
    "CONE_AVOID":     (30,  140, 255),
    "CROSSWALK":      (0,   210, 220),
    "EMERGENCY_STOP": (40,   60, 240),
    "_VRF_CONE":      (200, 160,  30),
    "_VRF_EMRG":      (200, 160,  30),
}
MISSION_LABEL = {
    "LANE_FOLLOW":    "LANE FOLLOW",
    "CONE_AVOID":     "CONE AVOID",
    "CROSSWALK":      "CROSSWALK STOP",
    "EMERGENCY_STOP": "EMERGENCY STOP",
    "_VRF_CONE":      "VERIFYING CONE...",
    "_VRF_EMRG":      "VERIFYING OBSTACLE...",
}
MISSION_DESC = {
    "LANE_FOLLOW":    "Lane following",
    "CONE_AVOID":     "Cone avoidance active",
    "CROSSWALK":      "Crosswalk stop",
    "EMERGENCY_STOP": "Emergency stop",
    "_VRF_CONE":      "LiDAR verifying...",
    "_VRF_EMRG":      "LiDAR verifying...",
}

_bridge = CvBridge()

# ── 전역 상태 ──────────────────────────────────────────────────────
g_camera_bgr    = None   # 원본 카메라
g_bev_bgr       = None   # BEV 차선 영상
g_mission       = "LANE_FOLLOW"
g_vlm_output    = "normal_drive"
g_vlm_raw       = ""
g_clusters      = []
g_start_time    = None

g_pub_viz       = None
g_video_writer  = None
g_p             = {}


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
    global g_mission
    g_mission = msg.data.strip()


def cb_vlm(msg):
    global g_vlm_output, g_vlm_raw
    try:
        j = json.loads(msg.data)
        g_vlm_output = j.get("candidate", "normal_drive")
        g_vlm_raw    = j.get("vlm_raw", "")[:55]
    except Exception:
        pass


def cb_clusters(msg):
    global g_clusters
    g_clusters = [(pt.x, pt.y) for pt in msg.points]


# ── 유틸 ──────────────────────────────────────────────────────────
def puttext(img, text, pos, scale=0.6, color=(255, 255, 255), thick=1):
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX,
                scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, thick, cv2.LINE_AA)


# ── LiDAR 미니맵 ──────────────────────────────────────────────────
def draw_lidar_map(clusters, size=190):
    mm = np.full((size, size, 3), (25, 25, 35), dtype=np.uint8)
    cx, cy  = size // 2, size - 20
    max_x   = 3.0
    scale   = (size - 30) / max_x

    for d in [0.5, 1.0, 2.0]:
        r = int(d * scale)
        cv2.circle(mm, (cx, cy), r, (55, 55, 70), 1)
        cv2.putText(mm, f"{d:.1f}m", (cx + r + 2, cy - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.28, (80, 80, 80), 1)
    cv2.line(mm, (cx, cy), (cx, 10), (55, 55, 70), 1)

    for (x, yr) in clusters:
        if x <= 0.0 or x > max_x:
            continue
        px = cx - int(yr * scale)
        py = cy - int(x  * scale)
        if 0 <= px < size and 0 <= py < size:
            color = (40, 60, 240) if np.hypot(x, yr) < 1.0 else (40, 180, 80)
            cv2.circle(mm, (px, py), 5, color, -1)

    cv2.circle(mm, (cx, cy), 7, (0, 220, 255), -1)
    return mm


# ── 우측 Decision 패널 ────────────────────────────────────────────
def build_panel(w, h):
    panel = np.full((h, w, 3), (18, 18, 28), dtype=np.uint8)
    now   = rospy.Time.now()
    t_rel = (now - g_start_time).to_sec() if g_start_time else 0.0

    y = 0

    # 타이틀
    cv2.rectangle(panel, (0, y), (w, y + 38), (32, 32, 50), -1)
    puttext(panel, "Decision", (8, y + 26), 0.85, (220, 220, 255), 2)
    puttext(panel, f"{int(t_rel//60):02d}:{t_rel%60:05.2f}",
            (w - 90, y + 26), 0.55, (160, 160, 160), 1)
    y += 42

    # 미션 배지
    mc  = MISSION_COLOR.get(g_mission, (120, 120, 120))
    lbl = MISSION_LABEL.get(g_mission, g_mission)
    cv2.rectangle(panel, (0, y), (w, y + 52), tuple(c // 4 for c in mc), -1)
    cv2.rectangle(panel, (0, y), (w, y + 52), mc, 2)
    ts  = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)[0]
    puttext(panel, lbl, ((w - ts[0]) // 2, y + 36), 0.9, mc, 2)
    y += 58

    # VLM 섹션
    cv2.rectangle(panel, (0, y), (w, y + 24), (30, 60, 30), -1)
    puttext(panel, "VLM  (Moondream2)", (6, y + 17), 0.55, (180, 240, 180), 1)
    y += 28

    out_color = (80, 220, 100) if g_vlm_output == "normal_drive" else (50, 150, 255)
    puttext(panel, f"output : {g_vlm_output}", (8, y + 16), 0.52, out_color)
    y += 22

    raw_lines = [g_vlm_raw[i:i+34] for i in range(0, min(len(g_vlm_raw), 68), 34)]
    for ln in raw_lines:
        puttext(panel, f'  "{ln}"', (8, y + 14), 0.44, (190, 190, 190))
        y += 17
    y += 8

    # Decision FSM 섹션
    cv2.rectangle(panel, (0, y), (w, y + 24), (30, 50, 80), -1)
    puttext(panel, "Decision FSM", (6, y + 17), 0.55, (160, 200, 255), 1)
    y += 28

    mc2 = MISSION_COLOR.get(g_mission, (180, 180, 180))
    puttext(panel, f"state : {g_mission}", (8, y + 16), 0.53, mc2)
    y += 22

    desc_color = MISSION_COLOR.get(g_mission, (180, 180, 180))
    desc       = MISSION_DESC.get(g_mission, "")
    if desc:
        puttext(panel, f"  {desc}", (8, y + 14), 0.47, desc_color)
    y += 22

    # LiDAR 섹션
    y += 6
    cv2.rectangle(panel, (0, y), (w, y + 24), (55, 30, 75), -1)
    puttext(panel, "LiDAR  (DBSCAN)", (6, y + 17), 0.55, (210, 160, 255), 1)
    y += 28

    mm_size = min(w - 16, h - y - 50)
    mm_size = max(100, min(190, mm_size))
    mm = draw_lidar_map(g_clusters, mm_size)
    mx = (w - mm_size) // 2
    if y + mm_size <= h:
        panel[y:y + mm_size, mx:mx + mm_size] = mm
        y += mm_size + 4

    n = len(g_clusters)
    puttext(panel, f"clusters : {n}", (8, y + 15), 0.5, (200, 200, 200))
    y += 20
    if g_clusters:
        nearest = min(np.hypot(x, yr) for x, yr in g_clusters)
        nc = (40, 60, 240) if nearest < 0.8 else (200, 200, 200)
        puttext(panel, f"nearest  : {nearest:.2f} m", (8, y + 15), 0.5, nc)

    return panel


# ── 이미지 서브윈도우 붙여넣기 ────────────────────────────────────
def paste_fit(canvas, img, x0, y0, w, h, label=None):
    """img를 (x0,y0) 위치의 w×h 박스에 비율 유지하며 붙여넣기"""
    if img is None:
        cv2.rectangle(canvas, (x0, y0), (x0 + w, y0 + h), (35, 35, 45), -1)
        puttext(canvas, "no signal", (x0 + w//2 - 40, y0 + h//2),
                0.55, (100, 100, 100))
        if label:
            puttext(canvas, label, (x0 + 6, y0 + 22), 0.6, (140, 140, 140))
        return
    ih, iw = img.shape[:2]
    scale  = min(w / iw, h / ih)
    nw, nh = int(iw * scale), int(ih * scale)
    ox = x0 + (w - nw) // 2
    oy = y0 + (h - nh) // 2
    canvas[oy:oy + nh, ox:ox + nw] = cv2.resize(img, (nw, nh))
    if label:
        cv2.rectangle(canvas, (x0, y0), (x0 + len(label)*9 + 12, y0 + 26),
                      (0, 0, 0), -1)
        puttext(canvas, label, (x0 + 6, y0 + 18), 0.55, (200, 200, 200))


# ── 미션 배지 오버레이 ────────────────────────────────────────────
def draw_mission_badge(canvas, x0, y0, w):
    mc  = MISSION_COLOR.get(g_mission, (120, 120, 120))
    lbl = MISSION_LABEL.get(g_mission, g_mission)
    ovl = canvas.copy()
    cv2.rectangle(ovl, (x0, y0), (x0 + w, y0 + 42), tuple(c // 3 for c in mc), -1)
    cv2.addWeighted(ovl, 0.7, canvas, 0.3, 0, canvas)
    ts = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.85, 2)[0]
    puttext(canvas, lbl, (x0 + (w - ts[0]) // 2, y0 + 30), 0.85, mc, 2)


# ── 프레임 합성 ───────────────────────────────────────────────────
def render_frame():
    W, H    = g_p["width"], g_p["height"]
    panel_w = 360
    cam_w   = W - panel_w   # 920px
    half_w  = cam_w // 2    # 460px (원본 | BEV)

    frame = np.zeros((H, W, 3), dtype=np.uint8)

    # 원본 카메라 (왼쪽)
    paste_fit(frame, g_camera_bgr, 0, 0, half_w, H, label="Camera")

    # BEV (오른쪽)
    paste_fit(frame, g_bev_bgr,   half_w, 0, half_w, H, label="BEV")

    # 중앙 구분선
    cv2.rectangle(frame, (half_w - 1, 0), (half_w + 1, H), (55, 55, 75), -1)

    # 미션 배지 (원본 카메라 상단)
    draw_mission_badge(frame, 0, 0, half_w)

    # 패널 구분선
    cv2.rectangle(frame, (cam_w - 1, 0), (cam_w + 1, H), (55, 55, 75), -1)

    # 우측 Decision 패널
    frame[:, cam_w:W] = build_panel(panel_w, H)

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
        rospy.loginfo("[demo_viz] 저장 완료: %s", g_p["output_path"])


# ── main ──────────────────────────────────────────────────────────
def main():
    global g_pub_viz, g_video_writer, g_p

    rospy.init_node("demo_viz_node")
    g_p = {
        "save_video":  bool(rospy.get_param("~save_video",   False)),
        "output_path": str(rospy.get_param("~output_path",   "/tmp/demo_output.mp4")),
        "fps":         float(rospy.get_param("~fps",         20.0)),
        "width":       int(rospy.get_param("~width",         1280)),
        "height":      int(rospy.get_param("~height",        720)),
    }

    g_pub_viz = rospy.Publisher("/demo/viz", Image, queue_size=1)

    if g_p["save_video"]:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        g_video_writer = cv2.VideoWriter(
            g_p["output_path"], fourcc,
            g_p["fps"], (g_p["width"], g_p["height"])
        )
        rospy.loginfo("[demo_viz] MP4 저장: %s", g_p["output_path"])

    rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage,
                     cb_camera,   queue_size=1, buff_size=2 ** 24)
    rospy.Subscriber("/perception/bev", Image,
                     cb_bev,      queue_size=1, buff_size=2 ** 24)
    rospy.Subscriber("/decision/mission", String,     cb_mission,  queue_size=1)
    rospy.Subscriber("/vlm/mission",      String,     cb_vlm,      queue_size=1)
    rospy.Subscriber("/lidar/clusters",   PointCloud, cb_clusters, queue_size=1)

    rospy.on_shutdown(on_shutdown)
    rospy.Timer(rospy.Duration(1.0 / g_p["fps"]), timer_cb)

    rospy.loginfo("[demo_viz] ready — rqt_image_view /demo/viz")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
