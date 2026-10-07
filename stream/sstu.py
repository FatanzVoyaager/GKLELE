#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
stu.py
Versi Python dari stu.cpp.

Fungsi utama:
- Membaca telemetry LAT/LON dari npord melalui UDP 127.0.0.1:5000.
- Mengaktifkan deteksi hanya ketika UAV berada <= 200 m dari salah satu target.
- Membuka kamera V4L2 /dev/video0..9 otomatis.
- Deteksi target BLUE/ORANGE menggunakan HSV + contour filtering.
- Tracking ROI setelah beberapa frame terdeteksi.
- Fail-safe: telemetry stale -> detection OFF.
- Recording MJPG 640x480 @ 30 FPS ke record/Nasional_YYYYmmdd_HHMMSS.mkv.
- Mengirim video sebagai RTP/H.264 UDP ke DEST_IP:PORT.

Pemakaian:
    python3 stu.py 192.168.1.100
    python3 stu.py 192.168.1.100 5600
    python3 stu.py 10.244.161.73 5600

Dependensi Python:
    python3-opencv
    python3-numpy
    python3-gi

GStreamer:
    gstreamer1.0-tools
    gstreamer1.0-plugins-base
    gstreamer1.0-plugins-good
    gstreamer1.0-plugins-bad
    gstreamer1.0-plugins-ugly
"""

import os
import sys
import time
import math
import signal
import socket
import threading
import subprocess
from pathlib import Path
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple

try:
    import cv2
except ImportError as exc:
    raise SystemExit("[ERROR] OpenCV Python belum terpasang: python3-opencv") from exc
try:
    import numpy as np
except ImportError as exc:
    raise SystemExit("[ERROR] NumPy belum terpasang: python3-numpy") from exc
try:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
except Exception as exc:
    raise SystemExit(
        "[ERROR] GObject/GStreamer Python belum tersedia. "
        "Pasang python3-gi dan paket GStreamer yang diperlukan."
    ) from exc

# =============================================================
# PARAMETER UTAMA - DISELARASKAN DENGAN stu.cpp
# =============================================================
WIDTH = 640
HEIGHT = 480
FPS = 30
DETECTION_TARGETS = [
    (-0.9610288, 116.7048257),
    (-0.8992702, 116.7678433),
    (-0.9103463, 116.8101063),
    (-0.9319386, 116.8497276),
]
DETECTION_RADIUS_M = 200.0
TELEMETRY_BIND_IP = "127.0.0.1"
TELEMETRY_PORT = 5000
TELEMETRY_TIMEOUT = 10.0
SDELAY = 2
BUFF_SEC = 1
ROTATE_SEC = 3600
LOST_LIMIT = 10
CAMERA_MIN = 0
CAMERA_MAX = 9
RECORD_DIR = Path("record")
RECORD_PREFIX = "Nasional"
# Pagi = true seperti stu.cpp. Untuk mengubah preset, ubah tiga flag ini.
PAGI = True
SIANG = False
SORE = False

# =============================================================
# GLOBAL STATE
# =============================================================
run_event = threading.Event()
run_event.set()
state_lock = threading.Lock()
current_lat = float("nan")
current_lon = float("nan")
telemetry_valid = False
detection_enable = False
last_telemetry_time = 0.0
current_detection_distance = float("nan")
appsrc = None
stream_pipeline = None
numframe = 0
appsrc_lock = threading.Lock()

# =============================================================
# UTILITAS
# =============================================================
def monotonic() -> float:
    return time.monotonic()

def finite(value: float) -> bool:
    return math.isfinite(value)

def device_exists(index: int) -> bool:
    path = f"/dev/video{index}"
    try:
        return os.path.exists(path) and os.path.exists(path)
    except OSError:
        return False

def angle(p1: Tuple[int, int], p2: Tuple[int, int], p0: Tuple[int, int]) -> float:
    dx1 = p1[0] - p0[0]
    dy1 = p1[1] - p0[1]
    dx2 = p2[0] - p0[0]
    dy2 = p2[1] - p0[1]
    dot = dx1 * dx2 + dy1 * dy2
    mag1 = math.hypot(dx1, dy1)
    mag2 = math.hypot(dx2, dy2)
    return dot / (mag1 * mag2 + 1e-10)

def haversine_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = (math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2)
    a = max(0.0, min(1.0, a))
    return 2.0 * r * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))

def get_percentile(channel: np.ndarray, percent: float) -> int:
    """Padanan getPercentile() dari C++; tidak dipakai oleh pipeline utama."""
    if channel.dtype != np.uint8 or channel.ndim != 2:
        raise ValueError("channel harus CV_8UC1 ekuivalen: uint8 grayscale")
    hist = cv2.calcHist([channel], [0], None, [256], [0, 256]).ravel()
    threshold = int(percent * channel.size)
    cumulative = 0
    for i, value in enumerate(hist):
        cumulative += int(value)
        if cumulative >= threshold:
            return i
    return 255

def calculate_detection_gate(lat: float, lon: float, now: float) -> Tuple[bool, int, float]:
    global current_detection_distance
    with state_lock:
        last = last_telemetry_time
        valid = telemetry_valid
    fresh = valid and last > 0.0 and (now - last <= TELEMETRY_TIMEOUT)
    if not fresh or not finite(lat) or not finite(lon):
        with state_lock:
            current_detection_distance = float("nan")
        return False, -1, float("nan")

    nearest_target = -1
    nearest_distance = float("inf")
    for i, (target_lat, target_lon) in enumerate(DETECTION_TARGETS, start=1):
        distance = haversine_distance_m(lat, lon, target_lat, target_lon)
        if distance < nearest_distance:
            nearest_distance = distance
            nearest_target = i
    with state_lock:
        current_detection_distance = nearest_distance
    return nearest_distance <= DETECTION_RADIUS_M, nearest_target, nearest_distance

# =============================================================
# FPS COUNTER
# =============================================================
class FPSCounter:
    def __init__(self) -> None:
        self.count = 0
        self.start = monotonic()

    def update(self) -> None:
        self.count += 1
        now = monotonic()
        sec = now - self.start
        if sec >= 1.0:
            print(f"[FPS] {self.count / sec:.2f}")
            self.count = 0
            self.start = now

fps = FPSCounter()

# =============================================================
# TELEMETRY UDP
# =============================================================
def telemetry_udp_loop() -> None:
    global current_lat, current_lon
    global telemetry_valid, detection_enable, last_telemetry_time
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(0.2)
    try:
        sock.bind((TELEMETRY_BIND_IP, TELEMETRY_PORT))
    except OSError as exc:
        print(f"[UDP] Bind {TELEMETRY_BIND_IP}:{TELEMETRY_PORT} gagal: {exc}")
        sock.close()
        return
    print(f"[UDP] Menunggu telemetry npord di " f"{TELEMETRY_BIND_IP}:{TELEMETRY_PORT}")
    while run_event.is_set():
        try:
            packet, _addr = sock.recvfrom(512)
        except socket.timeout:
            continue
        except OSError as exc:
            if run_event.is_set():
                print(f"[UDP] recvfrom gagal: {exc}")
            break
        try:
            message = packet.decode("ascii", errors="strict").strip()
        except UnicodeDecodeError:
            print("[UDP] Paket telemetry bukan ASCII")
            continue
        # Format dari npord: LAT=<latitude> LON=<longitude>
        parts = message.split()
        if len(parts) != 2 or not parts[0].startswith("LAT=") or not parts[1].startswith("LON="):
            print(f"[UDP] Paket telemetry tidak valid: {message}")
            continue

        try:
            lat = float(parts[0].split("=", 1)[1])
            lon = float(parts[1].split("=", 1)[1])
        except (ValueError, IndexError):
            print(f"[UDP] Paket telemetry tidak valid: {message}")
            continue
        if not finite(lat) or not finite(lon):
            print(f"[UDP] LAT/LON tidak finite: {message}")
            continue

        now = monotonic()
        with state_lock:
            current_lat = lat
            current_lon = lon
            telemetry_valid = True
            last_telemetry_time = now
        enabled, target_id, distance = calculate_detection_gate(lat, lon, now)
        with state_lock:
            previous = detection_enable
            detection_enable = enabled
        if enabled != previous:
            state = "ON" if enabled else "OFF"
            print(f"[DETECT] {state} | TARGET={target_id} | " f"DIST={distance:.2f} m | RADIUS={DETECTION_RADIUS_M:.2f} m | " f"LAT={lat:.7f} LON={lon:.7f}")
    with state_lock:
        detection_enable = False
        telemetry_valid = False
    sock.close()
    print("[UDP] Telemetry thread berhenti")

def telemetry_snapshot() -> Tuple[float, float, bool, float, float]:
    with state_lock:
        return (current_lat, current_lon, telemetry_valid, last_telemetry_time, current_detection_distance)

def detection_snapshot() -> bool:
    with state_lock:
        return detection_enable

# =============================================================
# CAMERA
# =============================================================
def configure_camera(cap: cv2.VideoCapture) -> None:
    # Sama seperti C++: MJPG + 640x480 + 30 FPS.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

def open_camera() -> Tuple[Optional[cv2.VideoCapture], int]:
    for idx in range(CAMERA_MIN, CAMERA_MAX + 1):
        if not device_exists(idx):
            continue

        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if cap.isOpened():
            configure_camera(cap)
            print(f"[INFO] Kamera ditemukan di /dev/video{idx}")
            return cap, idx
        
        cap.release()
    return None, -1

def reset_uvc_driver() -> None:
    print("[RECOVERY] Reset driver kamera")
    try:
        subprocess.run(["sudo", "modprobe", "-r", "uvcvideo"], check=False, timeout=8)
        time.sleep(0.5)
        subprocess.run(["sudo", "modprobe", "uvcvideo"], check=False, timeout=8)
    except (OSError, subprocess.SubprocessError) as exc:
        print(f"[WARN] Reset uvcvideo gagal: {exc}")

def create_writer(start_time_holder: list) -> cv2.VideoWriter:
    RECORD_DIR.mkdir(parents=True, exist_ok=True)
    now = time.localtime()
    filename = RECORD_DIR / time.strftime(f"{RECORD_PREFIX}_%Y%m%d_%H%M%S.mkv", now)
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    writer = cv2.VideoWriter(str(filename), fourcc, FPS, (WIDTH, HEIGHT))
    if not writer.isOpened():
        print(f"[ERROR] Writer gagal: {filename}")
    else:
        print(f"[INFO] Recording: {filename}")

    start_time_holder[0] = time.time()
    return writer

def get_hsv_ranges() -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    # Default values dari stu.cpp.
    blh, bls, blv = 90, 75, 145
    buh, bus, buv = 130, 255, 255

    olh1, ols1, olv1 = 0, 46, 176
    ouh1, ous1, ouv1 = 30, 255, 255

    olh2, ols2, olv2 = 0, 46, 176
    ouh2, ous2, ouv2 = 30, 255, 255

    if PAGI:
        # Pagi 7 - 9
        blh, bls, blv = 90, 75, 145
        buh, bus, buv = 105, 255, 255

        olh1, ols1, olv1 = 165, 46, 176
        ouh1, ous1, ouv1 = 180, 255, 255

        olh2, ols2, olv2 = 0, 46, 176
        ouh2, ous2, ouv2 = 30, 255, 255

    if SIANG:
        # Siang 9 - 12
        blh, bls, blv = 90, 75, 145
        buh, bus, buv = 110, 255, 255

        olh1, ols1, olv1 = 160, 46, 176
        ouh1, ous1, ouv1 = 180, 255, 255

        olh2, ols2, olv2 = 0, 46, 176
        ouh2, ous2, ouv2 = 30, 255, 255

    if SORE:
        # Sore 13 - 16 / alias mendung
        blh, bls, blv = 90, 75, 145
        buh, bus, buv = 110, 255, 255

        olh1, ols1, olv1 = 165, 46, 176
        ouh1, ous1, ouv1 = 180, 255, 255

        olh2, ols2, olv2 = 0, 46, 176
        ouh2, ous2, ouv2 = 30, 255, 255

    blue_low = np.array([blh, bls, blv], dtype=np.uint8)
    blue_high = np.array([buh, bus, buv], dtype=np.uint8)
    orange_low1 = np.array([olh1, ols1, olv1], dtype=np.uint8)
    orange_high1 = np.array([ouh1, ous1, ouv1], dtype=np.uint8)
    orange_low2 = np.array([olh2, ols2, olv2], dtype=np.uint8)
    orange_high2 = np.array([ouh2, ous2, ouv2], dtype=np.uint8)
    # Dikembalikan packed supaya caller tidak perlu mengubah logika C++.
    return blue_low, blue_high, (orange_low1, orange_high1), (orange_low2, orange_high2)

def build_detection(frame: np.ndarray, roi_lock: bool, track_roi: Optional[Tuple[int, int, int, int]]) -> Tuple[bool, str, float, Optional[Tuple[float, float]], Optional[Tuple[int, int, int, int]], Optional[Tuple[int, int, int, int]], np.ndarray]:
    """Mengembalikan hasil deteksi dan frame yang sudah diberi overlay contour."""
    h, w = frame.shape[:2]
    cframe = w // 2
    detected = False
    target_color = "UNKNOWN"
    best_score = -1.0
    best_center = None
    best_rect = None
    status = "SEARCH"

    blur = cv2.medianBlur(frame, 3)
    blur = cv2.GaussianBlur(blur, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    blue_low, blue_high, orange1, orange2 = get_hsv_ranges()
    bmask = cv2.inRange(hsv, blue_low, blue_high)
    omask1 = cv2.inRange(hsv, orange1[0], orange1[1])
    omask2 = cv2.inRange(hsv, orange2[0], orange2[1])
    omask = cv2.bitwise_or(omask1, omask2)
    mask = cv2.bitwise_or(bmask, omask)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    roi_offset_x = 0
    roi_offset_y = 0
    proc_mask = mask
    if roi_lock and track_roi is not None:
        tx, ty, tw, th = track_roi
        x1 = max(0, tx)
        y1 = max(0, ty)
        x2 = min(w, tx + tw)
        y2 = min(h, ty + th)
        if x2 > x1 and y2 > y1:
            proc_mask = mask[y1:y2, x1:x2]
            roi_offset_x = x1
            roi_offset_y = y1
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
        else:
            roi_lock = False
            proc_mask = mask
    else:
        roi_lock = False

    contours, _ = cv2.findContours(proc_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if roi_lock:
        for cnt in contours:
            cnt[:, 0, 0] += roi_offset_x
            cnt[:, 0, 1] += roi_offset_y

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 150:
            continue

        if area > w * h * 0.6:
            continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)
        max_cos = 0.0
        if len(approx) > 0:
            for i in range(len(approx)):
                p1 = tuple(int(v) for v in approx[(i + 1) % len(approx)][0])
                p2 = tuple(int(v) for v in approx[(i - 1 + len(approx)) % len(approx)][0])
                p0 = tuple(int(v) for v in approx[i][0])
                cosv = abs(angle(p1, p2, p0))
                max_cos = max(max_cos, cosv)

        x, y, rw, rh = cv2.boundingRect(cnt)
        if rw < 10 or rh < 10:
            continue

        ratio = max(rw, rh) / (min(rw, rh) + 1e-10)
        extent = area / (rw * rh + 1e-10)
        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        solidity = area / hull_area if hull_area > 0 else 0.0
        circularity = (4.0 * math.pi * area) / (peri * peri + 1e-10)
        cmask = np.zeros((rh, rw), dtype=np.uint8)
        cnt_roi = cnt.copy()
        cnt_roi[:, 0, 0] -= x
        cnt_roi[:, 0, 1] -= y
        cv2.drawContours(cmask, [cnt_roi], -1, 255, -1)
        # BoundingRect pada contour yang sudah dikembalikan ke koordinat full-frame.
        bmask_roi = bmask[y:y + rh, x:x + rw]
        omask_roi = omask[y:y + rh, x:x + rw]
        if bmask_roi.shape != cmask.shape or omask_roi.shape != cmask.shape:
            continue

        bonly = cv2.bitwise_and(bmask_roi, cmask)
        oronly = cv2.bitwise_and(omask_roi, cmask)
        bpx = cv2.countNonZero(bonly)
        opx = cv2.countNonZero(oronly)
        blue_ratio = bpx / (area + 1e-10)
        orange_ratio = opx / (area + 1e-10)
        if blue_ratio > 0.5 and blue_ratio > orange_ratio:
            color = "BLUE"
        elif orange_ratio > 0.5 and orange_ratio > blue_ratio:
            color = "ORANGE"
        else:
            color = "UNKNOWN"

        ok = True
        if len(approx) < 4 or len(approx) > 6:
            ok = False
        elif max_cos > 0.88:
            ok = False
        elif circularity < 0.65:
            ok = False
        elif extent < 0.63:
            ok = False
        elif solidity < 0.60:
            ok = False
        elif ratio > 1.5:
            ok = False
        elif color == "UNKNOWN":
            ok = False

        if not ok:
            continue

        moments = cv2.moments(cnt)
        if moments["m00"] == 0:
            continue

        cx = moments["m10"] / moments["m00"]
        cy = moments["m01"] / moments["m00"]
        dist = abs(cx - cframe)
        score = area - (dist * 2.0)
        if score > best_score:
            best_score = score
            best_center = (cx, cy)
            best_rect = (x, y, rw, rh)
            target_color = color
            detected = True

        box_color = (255, 0, 0) if color == "BLUE" else (0, 0, 255) if color == "ORANGE" else (0, 255, 0)
        cv2.polylines(frame, [approx], True, box_color, 2)
        cv2.rectangle(frame, (x, y), (x + rw, y + rh), box_color, 2)
    new_roi = None
    if detected and best_center is not None and best_rect is not None:
        has_x, has_y = best_center
        x, y, rw, rh = best_rect
        status = "LOCKED" if abs(has_x - cframe) < 60 else "TRACKING"
        roi_size = int(max(rw, rh) * 3.5)
        rx = int(has_x - roi_size / 2)
        ry = int(has_y - roi_size / 2)
        rx = max(0, rx)
        ry = max(0, ry)
        if rx + roi_size > w:
            roi_size = w - rx

        if ry + roi_size > h:
            roi_size = min(roi_size, h - ry)

        if roi_size > 0:
            new_roi = (rx, ry, roi_size, roi_size)

        cv2.circle(frame, (int(has_x), int(has_y)), 5, (0, 0, 255), -1)
        cv2.rectangle(frame, (x, y), (x + rw, y + rh), (255, 0, 0), 2)
        cv2.putText(frame, target_color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    return detected, target_color, best_score, best_center, best_rect, new_roi, frame

def camera_loop() -> None:
    global appsrc, numframe
    cap: Optional[cv2.VideoCapture] = None
    cam_index = -1
    buff = deque(maxlen=FPS * BUFF_SEC)
    start_time_holder = [0.0]
    fwriter = create_writer(start_time_holder)
    ftime = start_time_holder[0]
    roi_lock = False
    track_roi: Optional[Tuple[int, int, int, int]] = None
    target_color = "UNKNOWN"
    lost_count = 0
    detect_count = 0
    last_frame_time = monotonic()
    fail_count = 0
    no_frame_count = 0
    try:
        while run_event.is_set():
            if cap is None or not cap.isOpened():
                if cap is not None:
                    cap.release()
                cap, cam_index = open_camera()
                if cap is None or not cap.isOpened():
                    fail_count += 1
                    print(f"[WARN] Tidak ada kamera ({fail_count})")
                    if fail_count > 5:
                        reset_uvc_driver()
                        fail_count = 0
                    time.sleep(0.5)

                    continue

                fail_count = 0
                no_frame_count = 0
                last_frame_time = monotonic()

            ok, frame = cap.read()
            if not ok or frame is None or frame.size == 0:
                no_frame_count += 1
                print(f"[WARN] Frame kosong ({no_frame_count}/10)")
                if no_frame_count > 10:
                    print("[ERROR] Kamera hang -> reconnect")
                    cap.release()
                    cap = None
                    no_frame_count = 0
                    time.sleep(0.5)
                
                continue

            no_frame_count = 0
            now = monotonic()
            if now - last_frame_time > 2.0:
                print("[ERROR] Kamera freeze -> reconnect paksa")
                cap.release()
                cap = None
                time.sleep(0.5)
                continue

            last_frame_time = now
            h, w = frame.shape[:2]
            cframe = w // 2
            status = "SEARCH"
            if not detection_snapshot():
                roi_lock = False
                track_roi = None
                lost_count = 0
                detect_count = 0
                target_color = "UNKNOWN"
                status = "DETECTION OFF"
                detected = False
                best_score = -1.0
            else:
                (detected, target_color, best_score, best_center, best_rect, new_roi, frame) = build_detection(frame, roi_lock, track_roi)
                if detected and best_center is not None and new_roi is not None:
                    lost_count = 0
                    detect_count += 1
                    if detect_count > 3 and best_score > 300:
                        roi_lock = True

                    if not roi_lock:
                        track_roi = new_roi
                    else:
                        if track_roi is None:
                            track_roi = new_roi
                        else:
                            tx, ty, tw, th = track_roi
                            nx, ny, nw, nh = new_roi
                            track_roi = (int(0.8 * tx + 0.2 * nx), int(0.8 * ty + 0.2 * ny), int(0.8 * tw + 0.2 * nw), int(0.8 * th + 0.2 * nh))
                else:
                    detect_count = 0
                    lost_count += 1
                    status = "LOST" if roi_lock else "SEARCH"
                    if lost_count > LOST_LIMIT:
                        roi_lock = False
                        track_roi = None
            # ---------------------------------------------------------
            # FAIL-SAFE telemetry stale
            # ---------------------------------------------------------
            lat, lon, telem_valid, last_telem, distance = telemetry_snapshot()
            telemetry_fresh = (telem_valid and last_telem > 0.0 and (now - last_telem <= TELEMETRY_TIMEOUT))
            if not telemetry_fresh:
                with state_lock:
                    detection_enable = False
                    current_detection_distance = float("nan")
                distance = float("nan")
            # ---------------------------------------------------------
            # Overlay
            # ---------------------------------------------------------
            cv2.line(frame, (cframe, 0), (cframe, h), (0, 0, 0), 2)
            if (telemetry_fresh and finite(lat) and finite(lon) and finite(distance)):
                telem_overlay = (f"LAT:{lat:.6f} LON:{lon:.6f} " f"D:{distance:.1f}m/R:{DETECTION_RADIUS_M:.1f}m " f"DET:{'ON' if detection_snapshot() else 'OFF'}")
            else:
                telem_overlay = (f"GPS : STALE D : ---/R : {DETECTION_RADIUS_M:.1f}m DET : OFF")

            cv2.putText(frame, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2)
            cv2.putText(frame, telem_overlay, (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2)
            fps.update()
            if fwriter.isOpened():
                fwriter.write(frame)

            buff.append(frame.copy())
            if time.time() - ftime > ROTATE_SEC:
                fwriter.release()
                fwriter = create_writer(start_time_holder)
                ftime = start_time_holder[0]

            push_frame_to_gstreamer(frame)
    finally:
        if cap is not None:
            cap.release()

        fwriter.release()
        end_appsrc_stream()
        print("[INFO] DONE")

# =============================================================
# GSTREAMER UDP/RTP H.264
# =============================================================
def make_udp_pipeline(destination_ip: str, port: int) -> str:
    return (
        "appsrc name=mysrc is-live=true format=time do-timestamp=true block=false "
        f"caps=video/x-raw,format=BGR,width={WIDTH},height={HEIGHT},framerate={FPS}/1 "
        "! videoconvert "
        "! video/x-raw,format=I420 "
        "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=1500 "
        "key-int-max=15 bframes=0 byte-stream=true "
        "! h264parse config-interval=1 "
        "! rtph264pay pt=96 config-interval=1 mtu=1200 "
        "! queue max-size-buffers=2 leaky=downstream "
        f"! udpsink host={destination_ip} port={port} sync=false async=false"
    )

def start_udp_streamer(destination_ip: str, port: int) -> bool:
    global stream_pipeline, appsrc, numframe
    Gst.init(None)
    pipeline_desc = make_udp_pipeline(destination_ip, port)
    print("[INFO] GStreamer pipeline:")
    print(pipeline_desc)
    try:
        pipeline_obj = Gst.parse_launch(pipeline_desc)
    except Exception as exc:
        print(f"[ERROR] GStreamer UDP pipeline gagal dibuat: {exc}")
        return False
    source = pipeline_obj.get_by_name("mysrc")
    if source is None:
        print("[ERROR] appsrc 'mysrc' tidak ditemukan")
        pipeline_obj.set_state(Gst.State.NULL)
        return False

    source.set_property("stream-type", 0)  # GST_APP_STREAM_TYPE_STREAM
    source.set_property("min-latency", 0)
    source.set_property("max-latency", 0)
    with appsrc_lock:
        stream_pipeline = pipeline_obj
        appsrc = source
        numframe = 0
    ret = pipeline_obj.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        print("[ERROR] UDP pipeline gagal masuk PLAYING")
        pipeline_obj.set_state(Gst.State.NULL)
        with appsrc_lock:
            stream_pipeline = None
            appsrc = None
        return False

    # Logging/error watch.
    bus = pipeline_obj.get_bus()
    if bus is not None:
        bus.add_signal_watch()
        bus.connect("message", on_gst_message)

    print("\n[INFO] UDP/RTP H.264 Stream Ready")
    print(f"Destination : {destination_ip}:{port}")
    print(f"Resolution  : {WIDTH}x{HEIGHT} @ {FPS} FPS")
    print("Codec       : H.264 x264 ultrafast, zerolatency")
    print("Protocol    : RTP/H.264 over UDP")
    print(f"QGC         : UDP video port {port}\n")
    return True

def on_gst_message(_bus, message) -> None:
    msg_type = message.type
    if msg_type == Gst.MessageType.ERROR:
        try:
            err, debug = message.parse_error()
            print(f"[GSTREAMER ERROR] {err}")
            if debug:
                print(f"[GSTREAMER DEBUG] {debug}")
        except Exception as exc:
            print(f"[GSTREAMER ERROR] parse error gagal: {exc}")
    elif msg_type == Gst.MessageType.WARNING:
        try:
            warn, debug = message.parse_warning()
            print(f"[GSTREAMER WARNING] {warn}")
            if debug:
                print(f"[GSTREAMER DEBUG] {debug}")
        except Exception:
            pass
    elif msg_type == Gst.MessageType.EOS:
        print("[GSTREAMER] EOS")

def push_frame_to_gstreamer(frame: np.ndarray) -> None:
    global numframe
    with appsrc_lock:
        source = appsrc
        frame_index = numframe
        if source is None:
            return
        
        numframe += 1
    data = np.ascontiguousarray(frame).tobytes()
    buffer = Gst.Buffer.new_allocate(None, len(data), None)
    if buffer is None:
        print("[ERROR] gst.Buffer allocation gagal")
        return

    buffer.fill(0, data)
    pts = Gst.util_uint64_scale(frame_index, Gst.SECOND, FPS)
    duration = Gst.util_uint64_scale(1, Gst.SECOND, FPS)
    buffer.pts = pts
    buffer.dts = pts
    buffer.duration = duration
    try:
        ret = source.emit("push-buffer", buffer)
        if ret not in (Gst.FlowReturn.OK, Gst.FlowReturn.LAST):
            # Jangan spam log setiap frame ketika pipeline sedang berhenti.
            if run_event.is_set():
                print(f"[GSTREAMER] push-buffer: {ret}")
    except Exception as exc:
        if run_event.is_set():
            print(f"[GSTREAMER] push-buffer exception: {exc}")

def end_appsrc_stream() -> None:
    with appsrc_lock:
        source = appsrc
    if source is not None:
        try:
            source.emit("end-of-stream")
        except Exception:
            pass

def stop_udp_streamer() -> None:
    global stream_pipeline, appsrc
    with appsrc_lock:
        pipeline_obj = stream_pipeline
        source = appsrc
        appsrc = None
        stream_pipeline = None
    if pipeline_obj is None:
        return

    try:
        if source is not None:
            try:
                source.emit("end-of-stream")
            except Exception:
                pass
        pipeline_obj.set_state(Gst.State.NULL)
    except Exception as exc:
        print(f"[WARN] Shutdown GStreamer: {exc}")

# =============================================================
# SIGNAL HANDLER
# =============================================================
def handle_signal(signum, _frame) -> None:
    print(f"\n[INFO] Shutdown signal {signum}...")
    run_event.clear()

def main() -> int:
    if len(sys.argv) < 2 or len(sys.argv) > 3:
        print(f"Usage: {sys.argv[0]} <DEST_IP> [PORT]")
        print(f"Example: {sys.argv[0]} 192.168.1.100 5600")
        print(f"Example ZeroTier: {sys.argv[0]} 10.244.161.73 5600")
        return 1

    destination_ip = sys.argv[1]
    port = 5600
    if len(sys.argv) == 3:
        try:
            port = int(sys.argv[2])
        except ValueError:
            print("[ERROR] Port harus berupa angka")
            return 1
        if not 1 <= port <= 65535:
            print("[ERROR] Port harus 1..65535")
            return 1

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    if not start_udp_streamer(destination_ip, port):
        run_event.clear()
        return 1

    telemetry_thread = threading.Thread(target=telemetry_udp_loop, name="telemetry", daemon=True)
    telemetry_thread.start()
    cam_thread = threading.Thread(target=camera_loop, name="camera", daemon=False)
    cam_thread.start()
    try:
        while run_event.is_set():
            time.sleep(0.2)
    except KeyboardInterrupt:
        run_event.clear()
    cam_thread.join()
    stop_udp_streamer()
    telemetry_thread.join(timeout=1.0)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
