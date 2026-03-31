#!/usr/bin/env python3
"""
camera_lane_perception.py
- BEV 기반 차선 인식 (인지 전용)
- 퍼블리시: center_point_px, lateral_offset, curvature, heading_error, lane_detected
- CLAHE 전처리 + HSV 이진화 + 슬라이딩 윈도우 + 2차 다항식 피팅
- CUDA 가속 (가능 시), 전부 rosparam으로 튜닝 가능
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
_bridge = CvBridge()
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, Bool

# ────────────────── CUDA 가용성 확인 ──────────────────
try:
    _gpu_available = cv2.cuda.getCudaEnabledDeviceCount() > 0
except Exception:
    _gpu_available = False

# ────────────────── 전역 퍼블리셔 ──────────────────
g_pub_center_point   = None
g_pub_lateral_offset = None
g_pub_curvature      = None
g_pub_heading_error  = None
g_pub_lane_detected  = None
g_pub_debug_img      = None

# ────────────────── 파라미터 (rosparam 로드) ──────────────────
g_p = {}

# ────────────────── 스무딩 / 트래킹 상태 ──────────────────
g_prev_left_poly  = None
g_prev_right_poly = None
g_prev_lateral    = 0.0
g_prev_heading    = 0.0
g_prev_curvature  = 0.0
g_frame_count     = 0


# ================================================================
#  파라미터 로드
# ================================================================
def load_params():
    global g_p
    p = rospy.get_param("~", {})

    # ROI (비율)
    g_p["roi_top_y"]    = float(p.get("roi_top_y_ratio",     0.60))
    g_p["roi_lt"]       = float(p.get("roi_left_top_ratio",  0.10))
    g_p["roi_rt"]       = float(p.get("roi_right_top_ratio", 0.80))
    g_p["roi_lb"]       = float(p.get("roi_left_bot_ratio", -0.45))
    g_p["roi_rb"]       = float(p.get("roi_right_bot_ratio", 1.40))

    # 슬라이딩 윈도우
    g_p["num_windows"]     = int(p.get("num_windows",     12))
    g_p["window_margin"]   = int(p.get("window_margin",   80))
    g_p["minpix_recenter"] = int(p.get("minpix_recenter", 50))
    g_p["min_lane_sep"]    = int(p.get("min_lane_sep",    80))

    # 차선 폭 (BEV px) — 한쪽만 검출 시 반대쪽 추정용
    g_p["lane_width_px"] = float(p.get("lane_width_px", 340.0))

    # HSV 하얀 차선
    g_p["h_low"]  = int(p.get("hsv_h_low",  0))
    g_p["s_low"]  = int(p.get("hsv_s_low",  0))
    g_p["v_low"]  = int(p.get("hsv_v_low",  200))
    g_p["h_high"] = int(p.get("hsv_h_high", 180))
    g_p["s_high"] = int(p.get("hsv_s_high", 40))
    g_p["v_high"] = int(p.get("hsv_v_high", 255))

    # HSV 노란 차선
    #g_p["h_low"]  = int(p.get("hsv_h_low",  18))
    #g_p["h_high"] = int(p.get("hsv_h_high", 38))
    #g_p["s_low"]  = int(p.get("hsv_s_low",  100))
    #g_p["s_high"] = int(p.get("hsv_s_high", 255))
    #g_p["v_low"]  = int(p.get("hsv_v_low",  110))
    #g_p["v_high"] = int(p.get("hsv_v_high", 230))
    
    # CLAHE
    g_p["clahe_clip"]  = float(p.get("clahe_clip_limit", 2.5))
    g_p["clahe_grid"]  = int(p.get("clahe_grid_size",    8))

    # 모폴로지
    g_p["morph_kernel"] = int(p.get("morph_kernel_size", 3))
    g_p["morph_iter"]   = int(p.get("morph_iterations",  1))

    g_p["xm_per_pix"] = float(p.get("xm_per_pix", 0.03))
    g_p["ym_per_pix"] = float(p.get("ym_per_pix", 0.05))

    # 다항식 피팅 차수 (2 = 2차 포물선)
    g_p["poly_order"] = int(p.get("poly_order", 2))

    # EMA smoothing (출력)
    g_p["ema_alpha"]   = float(p.get("ema_alpha", 0.3))
    # 다항식 추적 EMA
    g_p["poly_alpha"]  = float(p.get("poly_alpha", 0.4))

    # 디버그 이미지 퍼블리시
    g_p["publish_debug"] = bool(p.get("publish_debug", True))

    # 최소 차선 포인트 수 (이 이하면 미검출 판정)
    g_p["min_lane_points"] = int(p.get("min_lane_points", 4))

    rospy.loginfo("[perception] params loaded: %s", g_p)


# ================================================================
#  ROI 폴리곤
# ================================================================
def make_roi_polygon(h, w):
    y_top = int(h * g_p["roi_top_y"])
    y_bot = h - 1
    x_lt  = int(w * g_p["roi_lt"])
    x_rt  = int(w * g_p["roi_rt"])
    x_lb  = int(w * g_p["roi_lb"])
    x_rb  = int(w * g_p["roi_rb"])
    return np.array([[x_lb, y_bot],
                     [x_lt, y_top],
                     [x_rt, y_top],
                     [x_rb, y_bot]], dtype=np.float32)


# ================================================================
#  BEV 워프
# ================================================================
def warp_to_bev(bgr, roi_poly):
    h, w = bgr.shape[:2]
    src = roi_poly.copy()
    src[:, 1] = np.clip(src[:, 1], 0, h - 1)

    dst = np.array([[0, h - 1],
                    [0, 0],
                    [w - 1, 0],
                    [w - 1, h - 1]], dtype=np.float32)

    M = cv2.getPerspectiveTransform(src, dst)

    if _gpu_available:
        try:
            gpu_bgr = cv2.cuda_GpuMat()
            gpu_bgr.upload(bgr)
            gpu_bev = cv2.cuda.warpPerspective(gpu_bgr, M, (w, h),
                                               flags=cv2.INTER_LINEAR,
                                               borderMode=cv2.BORDER_CONSTANT)
            return gpu_bev.download()
        except Exception:
            pass

    return cv2.warpPerspective(bgr, M, (w, h),
                               flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_CONSTANT,
                               borderValue=(0, 0, 0))


# ================================================================
#  전처리 + 이진화
# ================================================================
def binarize_lanes(bgr):
    # CLAHE on V channel for lighting robustness
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    clahe = cv2.createCLAHE(clipLimit=g_p["clahe_clip"],
                            tileGridSize=(g_p["clahe_grid"], g_p["clahe_grid"]))
    hsv[:, :, 2] = clahe.apply(hsv[:, :, 2])

    lower = np.array([g_p["h_low"],  g_p["s_low"],  g_p["v_low"]])
    upper = np.array([g_p["h_high"], g_p["s_high"], g_p["v_high"]])
    mask = cv2.inRange(hsv, lower, upper)

    k = g_p["morph_kernel"]
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=g_p["morph_iter"])
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=g_p["morph_iter"])

    return mask


# ================================================================
#  슬라이딩 윈도우 → 좌/우 픽셀 수집
# ================================================================
def sliding_window(binary):
    h, w = binary.shape
    num_win = g_p["num_windows"]
    margin  = g_p["window_margin"]
    minpix  = g_p["minpix_recenter"]
    min_sep = g_p["min_lane_sep"]

    # 하단 절반 히스토그램으로 base 탐색
    histogram = np.sum(binary[h // 2:, :], axis=0)
    mid = w // 2

    left_base  = int(np.argmax(histogram[:mid]))   if np.max(histogram[:mid])  > 0 else -1
    right_base = int(np.argmax(histogram[mid:]) + mid) if np.max(histogram[mid:]) > 0 else -1

    nz_y, nz_x = binary.nonzero()
    win_h = h // num_win

    left_cur  = left_base
    right_cur = right_base

    left_pts_x  = []
    left_pts_y  = []
    right_pts_x = []
    right_pts_y = []

    for win_idx in range(num_win):
        y_lo = h - (win_idx + 1) * win_h
        y_hi = h - win_idx * win_h

        in_band = (nz_y >= y_lo) & (nz_y < y_hi)

        # left
        good_left = np.array([], dtype=int)
        if left_cur >= 0:
            in_left = in_band & (nz_x >= left_cur - margin) & (nz_x < left_cur + margin)
            good_left = np.where(in_left)[0]

        # right
        good_right = np.array([], dtype=int)
        if right_cur >= 0:
            in_right = in_band & (nz_x >= right_cur - margin) & (nz_x < right_cur + margin)
            good_right = np.where(in_right)[0]

        # 너무 가까우면 한쪽 억제
        if left_cur >= 0 and right_cur >= 0 and abs(left_cur - right_cur) < min_sep:
            if len(good_left) < len(good_right):
                good_left = np.array([], dtype=int)
            else:
                good_right = np.array([], dtype=int)

        if len(good_left) > 0:
            left_pts_x.append(nz_x[good_left])
            left_pts_y.append(nz_y[good_left])
            if len(good_left) >= minpix:
                left_cur = int(np.mean(nz_x[good_left]))

        if len(good_right) > 0:
            right_pts_x.append(nz_x[good_right])
            right_pts_y.append(nz_y[good_right])
            if len(good_right) >= minpix:
                right_cur = int(np.mean(nz_x[good_right]))

    # 합치기
    lx = np.concatenate(left_pts_x)  if left_pts_x  else np.array([])
    ly = np.concatenate(left_pts_y)  if left_pts_y  else np.array([])
    rx = np.concatenate(right_pts_x) if right_pts_x else np.array([])
    ry = np.concatenate(right_pts_y) if right_pts_y else np.array([])

    return lx, ly, rx, ry


# ================================================================
#  다항식 피팅  x = f(y) = a*y^2 + b*y + c
# ================================================================
def fit_lane_poly(pts_x, pts_y, prev_poly):
    global g_p
    order = g_p["poly_order"]
    alpha = g_p["poly_alpha"]
    min_pts = g_p["min_lane_points"]

    if len(pts_x) < min_pts:
        return prev_poly  # 부족하면 이전 값 유지 (None 가능)

    # x = f(y) 피팅 (y 기준 다항식)
    coeffs = np.polyfit(pts_y, pts_x, order)

    if prev_poly is not None and len(prev_poly) == len(coeffs):
        coeffs = alpha * np.array(prev_poly) + (1.0 - alpha) * coeffs

    return coeffs


# ================================================================
#  다항식으로부터 메트릭 계산
# ================================================================
def compute_lane_metrics(left_poly, right_poly, h, w):
    """
    Returns:
        center_x  : 차선 중심 x (BEV 하단 기준)
        center_y  : 차선 중심 y
        lateral   : 이미지 중심으로부터의 px 오프셋 (양수 = 우측 치우침)
        curvature : 곡률 (양수 = 좌회전)
        heading   : heading error [rad] (양수 = 차가 우측으로 틀어짐)
        detected  : 검출 여부
    """
    xm_per_pix = g_p["xm_per_pix"]
    ym_per_pix = g_p["ym_per_pix"]
    half_w = g_p["lane_width_px"] / 2.0
    img_cx = w / 2.0

    # 평가 y 좌표들
    y_eval_bottom = h - 1       # 차량 바로 앞
    y_eval_mid    = h * 0.5     # 중간

    has_left  = left_poly  is not None
    has_right = right_poly is not None

    if not has_left and not has_right:
        return 0, 0, 0, 0, 0, False

    # 하단에서 x 위치 계산
    if has_left:
        left_x_bot  = np.polyval(left_poly,  y_eval_bottom)
        left_x_mid  = np.polyval(left_poly,  y_eval_mid)
    if has_right:
        right_x_bot = np.polyval(right_poly, y_eval_bottom)
        right_x_mid = np.polyval(right_poly, y_eval_mid)

    # 차선 중심
    if has_left and has_right:
        cx_bot = (left_x_bot + right_x_bot) / 2.0
        cx_mid = (left_x_mid + right_x_mid) / 2.0
    elif has_left:
        cx_bot = left_x_bot + half_w
        cx_mid = left_x_mid + half_w
    else:
        cx_bot = right_x_bot - half_w
        cx_mid = right_x_mid - half_w

    center_x = cx_bot
    center_y = float(y_eval_bottom)
    lateral  = center_x - img_cx   # 양수 = 우측 치우침

    # 곡률 & heading — 포인트 많은 쪽 다항식 사용
    ref_poly = left_poly if has_left else right_poly
    if has_left and has_right:
        # 중심선 다항식 = 평균
        ref_poly = (np.array(left_poly) + np.array(right_poly)) / 2.0

    # x = a*y^2 + b*y + c  →  dx/dy = 2*a*y + b,  d2x/dy2 = 2*a
    #order = len(ref_poly) - 1
    #if order >= 2:
        #a = ref_poly[0]
        #b = ref_poly[1]

        #dxdy  = 2.0 * a * y_eval_bottom + b
        #d2xdy = 2.0 * a
        #denom = (1.0 + dxdy**2) ** 1.5
        #curvature = d2xdy / denom if abs(denom) > 1e-9 else 0.0
    #else:
        #b = ref_poly[0] if order >= 1 else 0.0
        #dxdy = b
        #curvature = 0.0

    xm_per_pix = g_p["xm_per_pix"]
    ym_per_pix = g_p["ym_per_pix"]

    a = ref_poly[0]
    b = ref_poly[1]

    # 노이즈 제거 (중요)
    if abs(a) < 1e-6:
        a = 0.0

    # pixel → meter 변환
    a_m = a * (xm_per_pix / (ym_per_pix**2))
    b_m = b * (xm_per_pix / ym_per_pix)

    y_m = y_eval_bottom * ym_per_pix

    dxdy  = 2.0 * a_m * y_m + b_m
    d2xdy = 2.0 * a_m

    curvature = d2xdy / (1 + dxdy**2)**1.5

    # heading error: BEV에서 차는 y축(아래→위) 방향 주행
    # dx/dy > 0 → 차선이 오른쪽으로 꺾임 → 차가 좌측으로 치우침
    heading = np.arctan(dxdy)  # rad

    return center_x, center_y, lateral, curvature, heading, True


# ================================================================
#  디버그 이미지 생성
# ================================================================
def draw_debug(bev_bgr, binary, left_poly, right_poly, center_x, center_y, lx, ly, rx, ry):
    h, w = binary.shape

    # 왼쪽: BEV 컬러에 차선 오버레이
    left_vis = bev_bgr.copy()
    plot_y = np.linspace(0, h - 1, 100).astype(int)
    if left_poly is not None:
        plot_lx = np.polyval(left_poly, plot_y).astype(int)
        for i in range(len(plot_y) - 1):
            if 0 <= plot_lx[i] < w and 0 <= plot_lx[i+1] < w:
                cv2.line(left_vis,
                         (plot_lx[i],   plot_y[i]),
                         (plot_lx[i+1], plot_y[i+1]),
                         (0, 0, 255), 2)
    if right_poly is not None:
        plot_rx = np.polyval(right_poly, plot_y).astype(int)
        for i in range(len(plot_y) - 1):
            if 0 <= plot_rx[i] < w and 0 <= plot_rx[i+1] < w:
                cv2.line(left_vis,
                         (plot_rx[i],   plot_y[i]),
                         (plot_rx[i+1], plot_y[i+1]),
                         (0, 255, 0), 2)
    cv2.circle(left_vis, (int(center_x), int(center_y)), 6, (255, 0, 255), -1)
    cv2.line(left_vis, (w // 2, 0), (w // 2, h - 1), (255, 255, 0), 1)
    cv2.putText(left_vis, "BEV", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # 오른쪽: 이진화 + 슬라이딩 윈도우 픽셀
    right_vis = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    if len(lx) > 0:
        right_vis[ly.astype(int), lx.astype(int)] = [0, 0, 255]
    if len(rx) > 0:
        right_vis[ry.astype(int), rx.astype(int)] = [0, 255, 0]
    cv2.putText(right_vis, "Binary", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    return np.hstack([left_vis, right_vis])


# ================================================================
#  EMA 스무딩
# ================================================================
def smooth(prev, cur, alpha):
    if prev is None:
        return cur
    return alpha * prev + (1.0 - alpha) * cur


# ================================================================
#  이미지 콜백
# ================================================================
def image_callback(msg):
    global g_prev_left_poly, g_prev_right_poly
    global g_prev_lateral, g_prev_heading, g_prev_curvature
    global g_frame_count

    # 디코딩
    np_arr = np.frombuffer(msg.data, np.uint8)
    bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    if bgr is None:
        return

    h, w = bgr.shape[:2]

    # 1) ROI → BEV
    roi_poly = make_roi_polygon(h, w)
    bev = warp_to_bev(bgr, roi_poly)

    # 2) 이진화
    binary = binarize_lanes(bev)

    # 3) 슬라이딩 윈도우
    lx, ly, rx, ry = sliding_window(binary)

    # 4) 다항식 피팅
    left_poly  = fit_lane_poly(lx, ly, g_prev_left_poly)
    right_poly = fit_lane_poly(rx, ry, g_prev_right_poly)
    g_prev_left_poly  = left_poly
    g_prev_right_poly = right_poly

    # 5) 메트릭 계산
    cx, cy, lateral, curvature, heading, detected = \
        compute_lane_metrics(left_poly, right_poly, h, w)

    # EMA 스무딩
    alpha = g_p["ema_alpha"]
    if detected:
        lateral   = smooth(g_prev_lateral,   lateral,   alpha)
        heading   = smooth(g_prev_heading,   heading,   alpha)
        curvature = smooth(g_prev_curvature, curvature, alpha)
        g_prev_lateral   = lateral
        g_prev_heading   = heading
        g_prev_curvature = curvature
    else:
        lateral   = g_prev_lateral
        heading   = g_prev_heading
        curvature = g_prev_curvature

    # 6) 퍼블리시
    stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()

    pt_msg = PointStamped()
    pt_msg.header.stamp    = stamp
    pt_msg.header.frame_id = "bev"
    pt_msg.point.x = float(cx)
    pt_msg.point.y = float(cy)
    g_pub_center_point.publish(pt_msg)

    g_pub_lateral_offset.publish(Float32(data=float(lateral)))
    g_pub_curvature.publish(Float32(data=float(curvature)))
    g_pub_heading_error.publish(Float32(data=float(heading)))
    g_pub_lane_detected.publish(Bool(data=detected))

    # 7) 디버그 이미지
    if g_p["publish_debug"] and g_pub_debug_img.get_num_connections() > 0:
        debug = draw_debug(bev, binary, left_poly, right_poly, cx, cy, lx, ly, rx, ry)
        img_msg = _bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        img_msg.header.stamp = stamp
        g_pub_debug_img.publish(img_msg)

    g_frame_count += 1


# ================================================================
#  main
# ================================================================
def main():
    global g_pub_center_point, g_pub_lateral_offset
    global g_pub_curvature, g_pub_heading_error
    global g_pub_lane_detected, g_pub_debug_img

    rospy.init_node("camera_lane_perception")
    load_params()

    # 퍼블리셔
    g_pub_center_point   = rospy.Publisher("/perception/center_point_px",
                                          PointStamped, queue_size=1)
    g_pub_lateral_offset = rospy.Publisher("/perception/lateral_offset",
                                          Float32, queue_size=1)
    g_pub_curvature      = rospy.Publisher("/perception/curvature",
                                          Float32, queue_size=1)
    g_pub_heading_error  = rospy.Publisher("/perception/heading_error",
                                          Float32, queue_size=1)
    g_pub_lane_detected  = rospy.Publisher("/perception/lane_detected",
                                          Bool, queue_size=1)
    g_pub_debug_img      = rospy.Publisher("/perception/debug",
                                          Image, queue_size=1)

    # 서브스크라이버
    rospy.Subscriber("/camera/color/image_raw/compressed",
                     CompressedImage, image_callback,
                     queue_size=1, buff_size=2**24)

    rospy.loginfo("[perception] camera_lane_perception ready  (CUDA=%s)", _gpu_available)
    rospy.spin()


if __name__ == "__main__":
    main()