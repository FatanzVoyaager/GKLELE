#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
str.py - Versi Python dari str.cpp

Fungsi utama:
- Membaca telemetry LAT/LON dari npord melalui UDP 127.0.0.1:5000.
- Mengaktifkan deteksi hanya ketika UAV berada <= DETECTION_RADIUS_M dari
  salah satu target koordinat.
- Membuka kamera /dev/video0..9 via V4L2 pada 640x480 @ 30 FPS.
- Deteksi objek warna BLUE/ORANGE menggunakan HSV + morphology.
- Tracking ROI sederhana dengan status SEARCH/TRACKING/LOCKED/LOST.
- Menampilkan overlay telemetry dan status deteksi.
- Recording MJPG 640x480 30 FPS ke record/Nasional_YYYYMMDD_HHMMSS.mkv.
- RTSP H.264 low-latency pada rtsp://<IP>:8554/live menggunakan appsrc.
- Fail-safe: telemetry stale > 10 s -> detection OFF.

Dependensi Ubuntu:
    sudo apt install python3-opencv python3-numpy python3-gi \
        gir1.2-gstreamer-1.0 gir1.2-gst-rtsp-server-1.0 \
        gstreamer1.0-tools gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly

Contoh:
    python3 str.py 0.0.0.0

RTSP:
    rtsp://0.0.0.0:8554/live

Catatan:
- Program ini TIDAK membuka /dev/ttyS7. npord yang mengirim LAT/LON ke UDP 5000.
- Jika GStreamer Python bindings tidak tersedia pada system Python yang dipakai,
  install paket distro python3-gi dan gir1.2-* yang sesuai versi Ubuntu.
"""

import argparse
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime
from typing import List, Optional, Tuple

import cv2
import numpy as np

try:
    import gi
    gi.require_version("Gst", "1.0")
    gi.require_version("GstRtspServer", "1.0")
    from gi.repository import Gst, GLib, GstRtspServer
except Exception as exc:  # pragma: no cover - environment dependent
    print("[ERROR] GStreamer Python bindings tidak tersedia:")
    print(f"        {exc}")
    print("[INFO] Install: python3-gi gir1.2-gstreamer-1.0 gir1.2-gst-rtsp-server-1.0")
    sys.exit(1)

# =============================================================
# KONFIGURASI VIDEO
# =============================================================
WIDTH = 640
HEIGHT = 480
FPS = 30
CAMERA_MAX_INDEX = 10

# =============================================================
# GATE DETEKSI BERDASARKAN KOORDINAT
# =============================================================
DETECTION_TARGETS: List[Tuple[float, float]] = [
    (-0.9610288, 116.7048257),  # Target 1
    (-0.8992702, 116.7678433),  # Target 2
    (-0.9103463, 116.8101063),  # Target 3
    (-0.9319386, 116.8497276),  # Target 4
]
DETECTION_RADIUS_M = 200.0
TELEMETRY_BIND_IP = "127.0.0.1"
TELEMETRY_PORT = 5000
TELEMETRY_TIMEOUT = 10.0

# =============================================================
# LAIN-LAIN
# =============================================================
SDELAY = 2
BUFF_SEC = 1
ROTATE_SEC = 3600
LOST_LIMIT = 10
COLOR_THRESHOLD = 0.5
RECORD_DIR = "record"
RECORD_PREFIX = "Nasional"

# Status global / shared antar-thread.
run_event = threading.Event()
run_event.set()
telemetry_lock = threading.Lock()
current_lat = math.nan
current_lon = math.nan
telemetry_valid = False
last_telemetry_time = 0.0
current_detection_distance = math.nan
detection_enable = False
appsrc_lock = threading.Lock()
global_appsrc: Optional[Gst.Element] = None
numframe = 0

# =============================================================
# UTILITAS
# =============================================================
def monotonic_time() -> float:
    return time.monotonic()

def is_finite(value: float) -> bool:
    return math.isfinite(value)

def haversine_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = (math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2)
    a = max(0.0, min(1.0, a))
    return 2.0 * r * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))

def calculate_detection_gate(lat: float, lon: float, now: float) -> Tuple[bool, int, float]:
    global current_detection_distance
    with telemetry_lock:
        last = last_telemetry_time
        valid = telemetry_valid
    fresh = valid and last > 0.0 and (now - last <= TELEMETRY_TIMEOUT)
    if not fresh or not is_finite(lat) or not is_finite(lon):
        return False, -1, math.nan

    nearest_target = -1
    nearest_distance = math.inf
    for idx, (target_lat, target_lon) in enumerate(DETECTION_TARGETS, start=1):
        distance = haversine_distance_m(lat, lon, target_lat, target_lon)
        if distance < nearest_distance:
            nearest_distance = distance
            nearest_target = idx
    with telemetry_lock:
        current_detection_distance = nearest_distance
    return nearest_distance <= DETECTION_RADIUS_M, nearest_target, nearest_distance

def update_detection_state_from_telemetry(lat: float, lon: float) -> None:
    global current_lat, current_lon
    global telemetry_valid, last_telemetry_time, detection_enable
    now = monotonic_time()
    enable, nearest_target, distance = calculate_detection_gate(lat, lon, now)
    with telemetry_lock:
        current_lat = lat
        current_lon = lon
        telemetry_valid = True
        last_telemetry_time = now
        current_detection_distance = distance
        previous = detection_enable
        detection_enable = enable
    if enable != previous:
        print(f"[DETECT] {'ON' if enable else 'OFF'} | " f"TARGET={nearest_target} | DIST={distance:.2f} m | " f"RADIUS={DETECTION_RADIUS_M:.2f} m | " f"LAT={lat:.7f} LON={lon:.7f}")

def set_detection_off() -> None:
    global telemetry_valid, detection_enable, current_detection_distance
    with telemetry_lock:
        telemetry_valid = False
        detection_enable = False
        current_detection_distance = math.nan

# =============================================================
# FPS COUNTER
# =============================================================
class FPSCounter:
    def __init__(self) -> None:
        self.count = 0
        self.start = monotonic_time()
        self.last_fps = 0.0

    def update(self) -> None:
        self.count += 1
        now = monotonic_time()
        sec = now - self.start
        if sec >= 1.0:
            self.last_fps = self.count / sec
            print(f"[FPS] {self.last_fps:.2f}")
            self.count = 0
            self.start = now

# =============================================================
# RECORDING
# =============================================================
def create_writer() -> Tuple[Optional[cv2.VideoWriter], float, str]:
    os.makedirs(RECORD_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(RECORD_DIR, f"{RECORD_PREFIX}_{stamp}.mkv")
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    writer = cv2.VideoWriter(filename, fourcc, FPS, (WIDTH, HEIGHT))
    if not writer.isOpened():
        print(f"[ERROR] Writer gagal: {filename}")
        return None, time.time(), filename

    print(f"[INFO] Recording: {filename}")
    return writer, time.time(), filename

# =============================================================
# KAMERA
# =============================================================
def open_camera() -> Tuple[Optional[cv2.VideoCapture], int]:
    for idx in range(CAMERA_MAX_INDEX):
        dev = f"/dev/video{idx}"
        if not os.path.exists(dev):
            continue

        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue

        print(f"[INFO] Kamera ditemukan di {dev}")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, FPS)
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        print(f"[INFO] Kamera mode: {actual_w}x{actual_h} @ {actual_fps:.2f} FPS")
        return cap, idx
    return None, -1

def reset_camera_driver() -> None:
    print("[RECOVERY] Reset driver kamera")
    try:
        subprocess.run(["sudo", "modprobe", "-r", "uvcvideo"], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.5)
        subprocess.run(["sudo", "modprobe", "uvcvideo"], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as exc:
        print(f"[WARN] Reset uvcvideo gagal: {exc}")

# =============================================================
# DETEKSI HSV
# =============================================================
def angle_cosine(p1: Tuple[int, int], p2: Tuple[int, int], p0: Tuple[int, int]) -> float:
    dx1 = p1[0] - p0[0]
    dy1 = p1[1] - p0[1]
    dx2 = p2[0] - p0[0]
    dy2 = p2[1] - p0[1]
    dot = dx1 * dx2 + dy1 * dy2
    mag1 = math.sqrt(dx1 * dx1 + dy1 * dy1)
    mag2 = math.sqrt(dx2 * dx2 + dy2 * dy2)
    return dot / (mag1 * mag2 + 1e-10)

def get_hsv_thresholds() -> Tuple[Tuple[int, int, int], Tuple[int, int, int], Tuple[int, int, int], Tuple[int, int, int], Tuple[int, int, int], Tuple[int, int, int]]:
    # Dipertahankan sama dengan str.cpp: pagi aktif, siang/sore tidak aktif.
    pagi = True
    siang = False
    sore = False
    # Default
    blue_low = (90, 75, 145)
    blue_high = (130, 255, 255)
    orange_low1 = (0, 46, 176)
    orange_high1 = (30, 255, 255)
    orange_low2 = (0, 46, 176)
    orange_high2 = (30, 255, 255)

    if pagi:
        blue_low = (90, 75, 145)
        blue_high = (105, 255, 255)
        orange_low1 = (165, 46, 176)
        orange_high1 = (180, 255, 255)
        orange_low2 = (0, 46, 176)
        orange_high2 = (30, 255, 255)

    if siang:
        blue_low = (90, 75, 145)
        blue_high = (110, 255, 255)
        orange_low1 = (160, 46, 176)
        orange_high1 = (180, 255, 255)
        orange_low2 = (0, 46, 176)
        orange_high2 = (30, 255, 255)

    if sore:
        blue_low = (90, 75, 145)
        blue_high = (110, 255, 255)
        orange_low1 = (165, 46, 176)
        orange_high1 = (180, 255, 255)
        orange_low2 = (0, 46, 176)
        orange_high2 = (30, 255, 255)

    return blue_low, blue_high, orange_low1, orange_high1, orange_low2, orange_high2


def detect_target(frame: np.ndarray, roi_lock: bool, track_roi: Optional[Tuple[float, float, float, float]]) -> Tuple[bool, Optional[Tuple[float, float]], Optional[Tuple[int, int, int, int]], str, float, np.ndarray, Optional[Tuple[int, int, int, int]]]:
    h, w = frame.shape[:2]
    cframe = w // 2
    blur = cv2.medianBlur(frame, 3)
    blur = cv2.GaussianBlur(blur, (5, 5), 0)
    blue_low, blue_high, orange_low1, orange_high1, orange_low2, orange_high2 = get_hsv_thresholds()

    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    bmask = cv2.inRange(hsv, np.array(blue_low), np.array(blue_high))
    omask1 = cv2.inRange(hsv, np.array(orange_low1), np.array(orange_high1))
    omask2 = cv2.inRange(hsv, np.array(orange_low2), np.array(orange_high2))
    omask = cv2.bitwise_or(omask1, omask2)
    mask = cv2.bitwise_or(bmask, omask)
    
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    proc_mask = mask
    roi_offset = (0, 0)
    safe_roi: Optional[Tuple[int, int, int, int]] = None
    if roi_lock and track_roi is not None:
        tx, ty, tw, th = track_roi
        x = max(0, int(round(tx)))
        y = max(0, int(round(ty)))
        x2 = min(w, int(round(tx + tw)))
        y2 = min(h, int(round(ty + th)))
        if x2 > x and y2 > y:
            safe_roi = (x, y, x2 - x, y2 - y)
            proc_mask = mask[y:y2, x:x2]
            roi_offset = (x, y)
            cv2.rectangle(frame, (x, y), (x2, y2), (255, 255, 255), 2)
        else:
            roi_lock = False
            proc_mask = mask

    contours, _ = cv2.findContours(proc_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_full = []
    ox, oy = roi_offset
    for cnt in contours:
        if ox or oy:
            cnt = cnt.copy()
            cnt[:, 0, 0] += ox
            cnt[:, 0, 1] += oy

        contours_full.append(cnt)
    best_score = -1.0
    best_center: Optional[Tuple[float, float]] = None
    best_rect: Optional[Tuple[int, int, int, int]] = None
    best_color = "UNKNOWN"
    for cnt in contours_full:
        area = cv2.contourArea(cnt)
        if area < 150:
            continue
        
        if area > w * h * 0.6:
            continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)
        max_cos = 0.0
        for i in range(len(approx)):
            p1 = tuple(int(v) for v in approx[(i + 1) % len(approx)][0])
            p2 = tuple(int(v) for v in approx[(i - 1 + len(approx)) % len(approx)][0])
            p0 = tuple(int(v) for v in approx[i][0])
            cosv = abs(angle_cosine(p1, p2, p0))
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

        # Mask kontur di bounding box, setara dengan cmask pada C++.
        cmask = np.zeros((rh, rw), dtype=np.uint8)
        cnt_roi = cnt.copy()
        cnt_roi[:, 0, 0] -= x
        cnt_roi[:, 0, 1] -= y
        cv2.drawContours(cmask, [cnt_roi], -1, 255, -1)

        bmask_roi = bmask[y:y + rh, x:x + rw]
        omask_roi = omask[y:y + rh, x:x + rw]
        bonly = cv2.bitwise_and(bmask_roi, cmask)
        oronly = cv2.bitwise_and(omask_roi, cmask)

        bpx = cv2.countNonZero(bonly)
        orpx = cv2.countNonZero(oronly)
        blue_ratio = bpx / (area + 1e-10)
        orange_ratio = orpx / (area + 1e-10)

        color = "UNKNOWN"
        if blue_ratio > COLOR_THRESHOLD and blue_ratio > orange_ratio:
            color = "BLUE"
        elif orange_ratio > COLOR_THRESHOLD and orange_ratio > blue_ratio:
            color = "ORANGE"

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
        if abs(moments["m00"]) < 1e-12:
            continue

        cx = moments["m10"] / moments["m00"]
        cy = moments["m01"] / moments["m00"]
        dist = abs(cx - cframe)
        score = area - (dist * 2.0)
        if score > best_score:
            best_score = score
            best_center = (cx, cy)
            best_rect = (x, y, rw, rh)
            best_color = color

        if color == "BLUE":
            box_color = (255, 0, 0)
        elif color == "ORANGE":
            box_color = (0, 0, 255)
        else:
            box_color = (0, 255, 0)

        cv2.polylines(frame, [approx], True, box_color, 2)
        cv2.rectangle(frame, (x, y), (x + rw, y + rh), box_color, 2)
    detected = best_center is not None and best_rect is not None
    new_roi: Optional[Tuple[int, int, int, int]] = None
    if detected:
        cx, cy = best_center
        x, y, rw, rh = best_rect
        roi_size = int(max(rw, rh) * 3.5)
        rx = max(0, int(cx - roi_size / 2.0))
        ry = max(0, int(cy - roi_size / 2.0))
        roi_size_i = max(1, roi_size)
        if rx + roi_size_i > w:
            roi_size_i = w - rx
        
        if ry + roi_size_i > h:
            roi_size_i = h - ry
            
        if roi_size_i > 0:
            new_roi = (rx, ry, roi_size_i, roi_size_i)
    return detected, best_center, best_rect, best_color, best_score, frame, new_roi

# =============================================================
# TELEMETRY UDP THREAD
# =============================================================
def telemetry_udp_loop() -> None:
    global telemetry_valid, detection_enable, current_detection_distance
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(0.2)
        sock.bind((TELEMETRY_BIND_IP, TELEMETRY_PORT))
    except OSError as exc:
        print(f"[UDP] Bind {TELEMETRY_BIND_IP}:{TELEMETRY_PORT} gagal: {exc}")
        sock.close()
        set_detection_off()
        return
    print(f"[UDP] Menunggu telemetry npord di {TELEMETRY_BIND_IP}:{TELEMETRY_PORT}")
    try:
        while run_event.is_set():
            try:
                packet, _addr = sock.recvfrom(512)
            except socket.timeout:
                continue
            except OSError as exc:
                if run_event.is_set():
                    print(f"[UDP] recvfrom gagal: {exc}")
                continue
            try:
                text = packet.decode("ascii", errors="strict").strip()
            except UnicodeDecodeError:
                print("[UDP] Paket telemetry bukan ASCII")
                continue
            # Format dari npord:
            # LAT=<latitude> LON=<longitude>
            parts = text.split()
            lat = lon = math.nan
            try:
                if len(parts) >= 2 and parts[0].startswith("LAT=") and parts[1].startswith("LON="):
                    lat = float(parts[0][4:])
                    lon = float(parts[1][4:])
                else:
                    raise ValueError
            except ValueError:
                print(f"[UDP] Paket telemetry tidak valid: {text}")
                continue
            if not (is_finite(lat) and is_finite(lon)):
                print(f"[UDP] Paket telemetry tidak valid: {text}")
                continue

            update_detection_state_from_telemetry(lat, lon)
    finally:
        with telemetry_lock:
            telemetry_valid = False
            detection_enable = False
            current_detection_distance = math.nan
        sock.close()
        print("[UDP] Telemetry thread berhenti")

# =============================================================
# GSTREAMER RTSP
# =============================================================
class RTSPServer:
    def __init__(self, port: int = 8554) -> None:
        self.port = port
        self.server = GstRtspServer.RTSPServer()
        self.server.set_service(str(port))
        self.server.set_address("0.0.0.0")
        mounts = self.server.get_mount_points()
        factory = GstRtspServer.RTSPMediaFactory()
        launch = (
            "( appsrc name=mysrc is-live=true format=time do-timestamp=true block=true "
            f"caps=video/x-raw,format=BGR,width={WIDTH},height={HEIGHT},framerate={FPS}/1 "
            "! queue leaky=downstream "
            "! videoconvert "
            "! video/x-raw,format=NV12 "
            "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=1200 "
            "key-int-max=15 bframes=0 "
            "! h264parse config-interval=1 "
            "! rtph264pay pt=96 name=pay0 config-interval=1 )"
        )

        factory.set_launch(launch)
        factory.set_shared(True)
        factory.connect("media-configure", self._on_media_configure)
        mounts.add_factory("/live", factory)
        self.factory = factory
        self.attached_id = 0

    def _on_media_configure(self, _factory: GstRtspServer.RTSPMediaFactory, media: GstRtspServer.RTSPMedia) -> None:
        global global_appsrc, numframe
        element = media.get_element()
        with appsrc_lock:
            if global_appsrc is not None:
                try:
                    global_appsrc.set_state(Gst.State.NULL)
                except Exception:
                    pass
                global_appsrc = None

            source = element.get_by_name("mysrc")
            if source is None:
                print("[RTSP] ERROR: appsrc 'mysrc' tidak ditemukan")
                return

            global_appsrc = source
            try:
                global_appsrc.set_property("format", Gst.Format.TIME)
                global_appsrc.set_property("is-live", True)
                global_appsrc.set_property("block", True)
            except Exception as exc:
                print(f"[RTSP] Gagal set property appsrc: {exc}")
            numframe = 0
            print("[RTSP] appsrc media terhubung")

    def run(self, ip: str) -> None:
        self.server.set_address(ip)
        self.attached_id = self.server.attach(None)
        if self.attached_id == 0:
            raise RuntimeError(f"Gagal attach RTSP server pada {ip}:{self.port}")

        print("\n[INFO] RTSP Stream Ready")
        print(f"rtsp://{ip}:{self.port}/live")
        loop = GLib.MainLoop()
        try:
            loop.run()
        finally:
            if self.attached_id:
                try:
                    GLib.source_remove(self.attached_id)
                except Exception:
                    pass
                self.attached_id = 0

# =============================================================
# PUSH FRAME KE APPSRC
# =============================================================
def push_frame_to_rtsp(frame: np.ndarray) -> None:
    global numframe
    with appsrc_lock:
        appsrc = global_appsrc
        frame_number = numframe
        if appsrc is None:
            return
        numframe += 1

    if not frame.flags["C_CONTIGUOUS"]:
        frame = np.ascontiguousarray(frame)

    data = frame.tobytes()
    buffer = Gst.Buffer.new_allocate(None, len(data), None)
    if buffer is None:
        return

    buffer.fill(0, data)
    pts = Gst.util_uint64_scale(frame_number, Gst.SECOND, FPS)
    duration = Gst.SECOND // FPS
    buffer.pts = pts
    buffer.dts = pts
    buffer.duration = duration
    try:
        ret = appsrc.emit("push-buffer", buffer)
        if ret not in (Gst.FlowReturn.OK, Gst.FlowReturn.FLUSHING):
            # FLUSHING normal saat client disconnect / pipeline stop.
            if ret != Gst.FlowReturn.EOS:
                print(f"[RTSP] push-buffer: {ret.value_nick if hasattr(ret, 'value_nick') else ret}")
    except Exception as exc:
        print(f"[RTSP] push-buffer exception: {exc}")

# =============================================================
# CAMERA LOOP
# =============================================================
def camera_loop() -> None:
    global current_detection_distance
    cap: Optional[cv2.VideoCapture] = None
    cam_index = -1
    writer, writer_start, _ = create_writer()
    buff = deque(maxlen=FPS * BUFF_SEC)
    fps_counter = FPSCounter()
    best_center: Optional[Tuple[float, float]] = None
    best_rect: Optional[Tuple[int, int, int, int]] = None
    target_color = "UNKNOWN"
    detected = False
    roi_lock = False
    track_roi: Optional[Tuple[float, float, float, float]] = None
    lost_count = 0
    detect_count = 0
    last_frame_time = monotonic_time()
    fail_count = 0
    no_frame_count = 0
    try:
        while run_event.is_set():
            if cap is None or not cap.isOpened():
                cap, cam_index = open_camera()
                if cap is None or not cap.isOpened():
                    fail_count += 1
                    print(f"[WARN] Tidak ada kamera ({fail_count})")
                    if fail_count > 5:
                        reset_camera_driver()
                        fail_count = 0
                    time.sleep(0.5)
                    continue

                fail_count = 0
                no_frame_count = 0
                last_frame_time = monotonic_time()

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
            now = monotonic_time()
            if now - last_frame_time > 2.0:
                print("[ERROR] Kamera freeze -> reconnect paksa")
                cap.release()
                cap = None
                time.sleep(0.5)
                continue

            last_frame_time = now
            # Pastikan frame sesuai pipeline yang ditentukan C++.
            if frame.shape[1] != WIDTH or frame.shape[0] != HEIGHT:
                frame = cv2.resize(frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)

            h, w = frame.shape[:2]
            cframe = w // 2
            detected = False
            status = "SEARCH"
            best_score = -1.0
            with telemetry_lock:
                telem_fresh = (telemetry_valid and last_telemetry_time > 0.0 and (now - last_telemetry_time <= TELEMETRY_TIMEOUT))
                detect_enabled = detection_enable
                lat_overlay = current_lat
                lon_overlay = current_lon
                dist_overlay = current_detection_distance
            # Fail-safe jika npord berhenti / telemetry stale.
            if not telem_fresh:
                with telemetry_lock:
                    detection_enable = False
                    current_detection_distance = math.nan
                detect_enabled = False
                dist_overlay = math.nan

            if not detect_enabled:
                detected = False
                roi_lock = False
                track_roi = None
                lost_count = 0
                detect_count = 0
                target_color = "UNKNOWN"
                status = "DETECTION OFF"
            else:
                (detected, best_center, best_rect, target_color, best_score, frame, new_roi) = detect_target(frame, roi_lock, track_roi)
                if detected and best_center is not None and best_rect is not None:
                    lost_count = 0
                    detect_count += 1
                    if detect_count > 3 and best_score > 300:
                        roi_lock = True

                    cv2.circle(frame, (int(round(best_center[0])), int(round(best_center[1]))), 5, (0, 0, 255), -1)
                    x, y, rw, rh = best_rect
                    cv2.rectangle(frame, (x, y), (x + rw, y + rh), (255, 0, 0), 2)
                    cv2.putText(frame, target_color, (x, max(15, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    error = int(round(best_center[0] - cframe))
                    status = "LOCKED" if abs(error) < 60 else "TRACKING"
                    if new_roi is not None:
                        rx, ry, rw2, rh2 = new_roi
                        if not roi_lock:
                            track_roi = (rx, ry, rw2, rh2)
                        elif track_roi is not None:
                            tx, ty, tw, th = track_roi
                            track_roi = (0.8 * tx + 0.2 * rx, 0.8 * ty + 0.2 * ry, 0.8 * tw + 0.2 * rw2, 0.8 * th + 0.2 * rh2)
                else:
                    detect_count = 0
                    lost_count += 1
                    status = "LOST" if roi_lock else "SEARCH"
                    if lost_count > LOST_LIMIT:
                        roi_lock = False
                        track_roi = None
            # Re-check freshness immediately before overlay/push.
            with telemetry_lock:
                telemetry_fresh = (telemetry_valid and last_telemetry_time > 0.0 and (monotonic_time() - last_telemetry_time <= TELEMETRY_TIMEOUT))
                lat_overlay = current_lat
                lon_overlay = current_lon
                dist_overlay = current_detection_distance
                detect_overlay = detection_enable
                if not telemetry_fresh:
                    detection_enable = False
                    current_detection_distance = math.nan
                    dist_overlay = math.nan
                    detect_overlay = False
            cv2.line(frame, (cframe, 0), (cframe, h), (0, 0, 0), 2)
            if (telemetry_fresh and is_finite(lat_overlay) and is_finite(lon_overlay) and is_finite(dist_overlay)):
                telem_overlay = (f"LAT:{lat_overlay:.6f} LON:{lon_overlay:.6f} " f"D:{dist_overlay:.1f}m/R:{DETECTION_RADIUS_M:.1f}m " f"DET:{'ON' if detect_overlay else 'OFF'}")
            else:
                telem_overlay = f"GPS : STALE D : ---/R : {DETECTION_RADIUS_M:.1f}m DET : OFF"

            # Sesuai C++: teks digambar hitam dengan outline sederhana juga tidak diubah.
            cv2.putText(frame, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2)
            cv2.putText(frame, telem_overlay, (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2)
            fps_counter.update()
            if writer is not None and writer.isOpened():
                writer.write(frame)

            buff.append(frame.copy())
            if time.time() - writer_start > ROTATE_SEC:
                if writer is not None:
                    writer.release()
                writer, writer_start, _ = create_writer()

            # RTSP hanya menerima frame ketika client membuat media/appsrc.
            push_frame_to_rtsp(frame)
    finally:
        if cap is not None:
            cap.release()

        if writer is not None:
            writer.release()

        with appsrc_lock:
            appsrc = global_appsrc
            # Jangan emit EOS di sini secara agresif; client disconnect/RTSP factory
            # menangani lifecycle pipeline. Set reference None agar frame baru tidak dipush.
            if appsrc is not None:
                global_appsrc = None
        print("[INFO] DONE")
# =============================================================
# SIGNAL HANDLING
# =============================================================
def on_signal(_signum: int, _frame) -> None:
    print("\n[INFO] Shutdown...")
    run_event.clear()

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="STR Python - RTSP + Detection + Telemetry")
    parser.add_argument("ip", help="IP address tempat RTSP server listen, contoh 0.0.0.0")
    parser.add_argument("--port", type=int, default=8554, help="Port RTSP (default 8554)")
    return parser.parse_args()

def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    Gst.init(None)
    # Thread telemetry UDP dan kamera berjalan paralel seperti std::thread pada C++.
    telemetry_thread = threading.Thread(target=telemetry_udp_loop, name="telemetry", daemon=True)
    cam_thread = threading.Thread(target=camera_loop, name="camera", daemon=True)
    telemetry_thread.start()
    cam_thread.start()
    try:
        server = RTSPServer(args.port)
        server.run(args.ip)
    except KeyboardInterrupt:
        run_event.clear()
    except Exception as exc:
        print(f"[ERROR] RTSP server: {exc}", file=sys.stderr)
        run_event.clear()
        return 1
    finally:
        run_event.clear()
        telemetry_thread.join(timeout=3.0)
        cam_thread.join(timeout=3.0)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
