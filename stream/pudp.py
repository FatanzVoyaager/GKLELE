#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Versi Python dari cudp.cpp.

Fungsi:
- Streaming kamera melalui RTP/H.264 over UDP menggunakan GStreamer appsrc.
- Kamera OpenCV/V4L2, auto-detect /dev/video0 .. /dev/video9.
- Resolusi 640x360 @ 30 FPS.
- Recording MJPG ke record/Nasional_YYYYMMDD_HHMMSS.mkv.
- Buffer frame 1 detik.
- Rotasi file recording setiap 1 jam.
- Recovery jika kamera tidak ditemukan, frame kosong, atau kamera freeze.

Tidak menggunakan RTSP dan tidak menggunakan telemetry/Pixhawk.
Tujuan UDP:
    <DEST_IP> [PORT]

Default:
    PORT = 5600

Contoh:
    python3 cudp.py 192.168.1.100 5600
    python3 cudp.py 10.244.161.73 5600
"""

import os
import sys
import time
import signal
import threading
from collections import deque
from datetime import datetime
from pathlib import Path
import cv2
# ============================================================================
# GSTREAMER / PYGI
# ============================================================================
try:
    import gi
    gi.require_version("Gst", "1.0")
    gi.require_version("GstApp", "1.0")
    from gi.repository import Gst, GstApp
except ImportError as exc:
    print("[ERROR] Modul PyGObject/GStreamer tidak tersedia.")
    print(f"[DETAIL] {exc}")
    print(
        "Install dependency, misalnya:\n"
        "  sudo apt install python3-gi "
        "gir1.2-gstreamer-1.0 "
        "gstreamer1.0-tools "
        "gstreamer1.0-plugins-base "
        "gstreamer1.0-plugins-good "
        "gstreamer1.0-plugins-bad "
        "gstreamer1.0-plugins-ugly"
    )
    sys.exit(1)

# ============================================================================
# KONFIGURASI - SAMA DENGAN cudp.cpp
# ============================================================================
WIDTH = 640
HEIGHT = 360
FPS = 30
BUFF_SEC = 1
ROTATE_SEC = 3600
DEFAULT_PORT = 5600
CAMERA_SCAN_COUNT = 10
CAMERA_RECONNECT_DELAY = 0.5
NO_FRAME_LIMIT = 10
CAMERA_FREEZE_SEC = 2.0
DRIVER_RESET_AFTER = 5
RECORD_DIR = Path("record")
RECORD_PREFIX = "Nasional"

# ============================================================================
# GLOBAL STATE
# ============================================================================
run_event = threading.Event()
run_event.set()
global_appsrc = None
stream_pipeline = None
appsrc_lock = threading.Lock()
pipeline_lock = threading.Lock()
numframe = 0
numframe_lock = threading.Lock()

# ============================================================================
# FPS COUNTER
# ============================================================================
class FPSCounter:
    def __init__(self):
        self.count = 0
        self.start = time.monotonic()

    def update(self):
        self.count += 1
        now = time.monotonic()
        elapsed = now - self.start
        if elapsed >= 1.0:
            print(f"[FPS] {self.count / elapsed:.2f}")
            self.count = 0
            self.start = now

fps = FPSCounter()
# ============================================================================
# SIGNAL HANDLER
# ============================================================================
def on_signal(signum, _frame):
    if signum in (signal.SIGINT, signal.SIGTERM):
        print("\n[INFO] Shutdown...")
        run_event.clear()

# ============================================================================
# RECORDING
# ============================================================================
def create_writer(name: str):
    RECORD_DIR.mkdir(parents=True, exist_ok=True)
    now = time.time()
    dt = datetime.fromtimestamp(now)
    filename = RECORD_DIR / f"{name}_{dt:%Y%m%d_%H%M%S}.mkv"
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    writer = cv2.VideoWriter(str(filename), fourcc, FPS, (WIDTH, HEIGHT))
    if not writer.isOpened():
        print(f"[ERROR] Writer gagal: {filename}")
    else:
        print(f"[INFO] Recording: {filename}")
    return writer, now

# ============================================================================
# CAMERA DISCOVERY
# ============================================================================
def device_exists(index: int) -> bool:
    path = f"/dev/video{index}"
    try:
        return os.path.exists(path) and os.stat(path).st_rdev != 0
    except OSError:
        return False

def open_camera():
    for index in range(CAMERA_SCAN_COUNT):
        if not device_exists(index):
            continue

        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue

        print(f"[INFO] Kamera ditemukan di /dev/video{index}")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, FPS)
        return cap, index
    return None, -1

def reset_uvcvideo():
    print("[RECOVERY] Reset driver kamera")
    rc1 = os.system("sudo modprobe -r uvcvideo")
    if rc1 != 0:
        print(f"[WARN] modprobe -r uvcvideo gagal (rc={rc1})")

    time.sleep(0.5)
    rc2 = os.system("sudo modprobe uvcvideo")
    if rc2 != 0:
        print(f"[WARN] modprobe uvcvideo gagal (rc={rc2})")

# ============================================================================
# GSTREAMER STATE
# ============================================================================
def get_appsrc():
    with appsrc_lock:
        return global_appsrc

def set_appsrc(value):
    global global_appsrc

    with appsrc_lock:
        global_appsrc = value

def push_frame(frame):
    appsrc = get_appsrc()
    if appsrc is None:
        return False

    if frame is None or frame.size == 0:
        return False

    if not frame.flags["C_CONTIGUOUS"]:
        frame = frame.copy()

    data = frame.tobytes()
    with numframe_lock:
        global numframe
        current_frame = numframe
    try:
        gst_buffer = Gst.Buffer.new_allocate(None, len(data), None)
        if gst_buffer is None:
            print("[ERROR] Gagal membuat GstBuffer")
            return False

        gst_buffer.fill(0, data)
        pts = Gst.util_uint64_scale(current_frame, Gst.SECOND, FPS)
        gst_buffer.pts = pts
        gst_buffer.dts = pts
        gst_buffer.duration = Gst.SECOND // FPS
        ret = appsrc.emit("push-buffer", gst_buffer)
        if ret == Gst.FlowReturn.OK:
            with numframe_lock:
                numframe += 1
            return True

        print(f"[WARN] push-buffer gagal: {ret}")
        return False
    except Exception as exc:
        print(f"[ERROR] push-buffer exception: {exc}")
        return False

# ============================================================================
# UDP/RTP H.264 STREAMER
# ============================================================================
class UDPStreamer:
    def __init__(self, destination_ip: str, port: int):
        global stream_pipeline, numframe
        self.destination_ip = destination_ip
        self.port = port
        self.ready = False

        pipeline_description = (
            f"appsrc name=mysrc "
            f"is-live=true "
            f"format=time "
            f"do-timestamp=true "
            f"block=false "
            f"caps=video/x-raw,format=BGR,width={WIDTH},height={HEIGHT},framerate={FPS}/1 "
            f"! videoconvert "
            f"! video/x-raw,format=I420 "
            f"! x264enc tune=zerolatency speed-preset=ultrafast bitrate=1200 "
            f"key-int-max=15 bframes=0 byte-stream=true "
            f"! h264parse config-interval=1 "
            f"! rtph264pay pt=96 config-interval=1 mtu=1200 "
            f"! queue max-size-buffers=2 leaky=downstream "
            f"! udpsink host={destination_ip} port={port} sync=false async=false"
        )

        self.pipeline_description = pipeline_description
        print("[INFO] GStreamer pipeline:")
        print(pipeline_description)
        try:
            pipeline = Gst.parse_launch(pipeline_description)
            if pipeline is None:
                print("[ERROR] GStreamer UDP pipeline gagal dibuat")
                return
        except Exception as exc:
            print(f"[ERROR] GStreamer UDP pipeline gagal dibuat: {exc}")
            return
        appsrc = pipeline.get_by_name("mysrc")
        if appsrc is None:
            print("[ERROR] appsrc 'mysrc' tidak ditemukan")
            try:
                pipeline.set_state(Gst.State.NULL)
            except Exception:
                pass
            return
        
        try:
            appsrc.set_property("stream-type", GstApp.AppStreamType.STREAM)
        except Exception:
            # Beberapa versi binding GI menerima nilai enum melalui integer.
            try:
                appsrc.set_property("stream-type", 0)
            except Exception as exc:
                print(f"[WARN] Gagal set stream-type: {exc}")
        # Sama seperti gst_app_src_set_latency(appsrc, 0, 0)
        try:
            appsrc.set_property("min-latency", 0)
            appsrc.set_property("max-latency", 0)
        except Exception as exc:
            print(f"[WARN] Gagal set latency appsrc: {exc}")
        with pipeline_lock:
            stream_pipeline = pipeline
        set_appsrc(appsrc)
        with numframe_lock:
            numframe = 0
        ret = pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("[ERROR] UDP pipeline gagal masuk PLAYING")
            self.shutdown()
            return

        self.ready = True
        print("\n[INFO] UDP/RTP H.264 Stream Ready")
        print(f"Destination : {destination_ip}:{port}")
        print(f"Resolution  : {WIDTH}x{HEIGHT} @ {FPS} FPS")
        print("Codec       : H.264 x264 ultrafast, zerolatency")
        print("Protocol    : RTP/H.264 over UDP")
        print(f"QGC         : UDP video port {port}\n")

    def is_ready(self) -> bool:
        return self.ready and self.get_pipeline() is not None and get_appsrc() is not None

    def get_pipeline(self):
        with pipeline_lock:
            return stream_pipeline

    def shutdown(self):
        global stream_pipeline
        pipeline = self.get_pipeline()
        if pipeline is None:
            self.ready = False
            set_appsrc(None)
            return

        appsrc = get_appsrc()
        if appsrc is not None:
            try:
                appsrc.emit("end-of-stream")
            except Exception as exc:
                print(f"[WARN] end-of-stream gagal: {exc}")

        set_appsrc(None)
        try:
            pipeline.set_state(Gst.State.NULL)
        except Exception as exc:
            print(f"[WARN] Gagal set pipeline NULL: {exc}")
        with pipeline_lock:
            if stream_pipeline is pipeline:
                stream_pipeline = None
        self.ready = False

def camera_loop():
    cap = None
    cam_index = -1
    last_frame_time = time.monotonic()
    fail_count = 0
    no_frame_count = 0
    frame_buffer = deque(maxlen=FPS * BUFF_SEC)
    fwriter, ftime = create_writer("Nasional")
    try:
        while run_event.is_set():
            if cap is None or not cap.isOpened():
                cap, cam_index = open_camera()
                if cap is None or not cap.isOpened():
                    fail_count += 1
                    print(f"[WARN] Tidak ada kamera ({fail_count})")
                    if fail_count > 5:
                        reset_uvcvideo()
                        fail_count = 0

                    time.sleep(0.5)
                    continue

                fail_count = 0
                no_frame_count = 0
                last_frame_time = time.monotonic()

            ok, frame = cap.read()
            if not ok or frame is None or frame.size == 0:
                no_frame_count += 1
                print(f"[WARN] Frame kosong ({no_frame_count}/10)")
                if no_frame_count > NO_FRAME_LIMIT:
                    print("[ERROR] Kamera hang -> reconnect")
                    try:
                        cap.release()
                    except Exception:
                        pass
                    cap = None
                    no_frame_count = 0
                    time.sleep(0.5)
                continue

            no_frame_count = 0
            now = time.monotonic()
            if now - last_frame_time > CAMERA_FREEZE_SEC:
                print("[ERROR] Kamera freeze -> reconnect paksa")
                try:
                    cap.release()
                except Exception:
                    pass
                cap = None
                time.sleep(0.5)
                continue

            last_frame_time = now
            # ---------------------------------------------------------------
            # OPTIONAL RESIZE SAFETY
            # cudp.cpp sudah meminta 640x360, tetapi resize menjaga caps tetap
            # benar jika driver memberikan ukuran berbeda.
            # ---------------------------------------------------------------
            if frame.shape[1] != WIDTH or frame.shape[0] != HEIGHT:
                frame = cv2.resize(frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)
            # ---------------------------------------------------------------
            # FPS
            # ---------------------------------------------------------------
            fps.update()
            # ---------------------------------------------------------------
            # RECORDING
            # ---------------------------------------------------------------
            if fwriter is not None and fwriter.isOpened():
                try:
                    fwriter.write(frame)
                except Exception as exc:
                    print(f"[ERROR] Gagal menulis recording: {exc}")
            # ---------------------------------------------------------------
            # BUFFER
            # ---------------------------------------------------------------
            frame_buffer.append(frame.copy())
            if len(frame_buffer) > FPS * BUFF_SEC:
                frame_buffer.popleft()
            # ---------------------------------------------------------------
            # ROTATE RECORDING
            # ---------------------------------------------------------------
            if time.time() - ftime > ROTATE_SEC:
                if fwriter is not None:
                    fwriter.release()

                fwriter, ftime = create_writer("Nasional")
            # ---------------------------------------------------------------
            # PUSH TO RTP/UDP
            # ---------------------------------------------------------------
            push_frame(frame)
    except Exception as exc:
        print(f"[ERROR] camera_loop exception: {exc}")
    finally:
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass

        if fwriter is not None:
            try:
                fwriter.release()
            except Exception:
                pass

        print("[INFO] Camera loop DONE")

def main() -> int:
    if len(sys.argv) < 2 or len(sys.argv) > 3:
        print(f"Usage: {sys.argv[0]} <DEST_IP> [PORT]")
        print(f"Example: {sys.argv[0]} 192.168.1.100 5600")
        print(f"Example ZeroTier: {sys.argv[0]} 10.244.161.73 5600")
        return 1

    destination_ip = sys.argv[1]
    port = DEFAULT_PORT
    if len(sys.argv) == 3:
        try:
            port = int(sys.argv[2])
        except ValueError:
            print("[ERROR] Port harus berupa angka")
            return 1
        if port < 1 or port > 65535:
            print("[ERROR] Port harus 1..65535")
            return 1

    # ------------------------------------------------------------------------
    # INIT GSTREAMER
    # ------------------------------------------------------------------------
    Gst.init(None)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    print("===============================================================")
    print("      CUDP - CAMERA STREAM RTP/H.264 UDP")
    print("===============================================================")
    print(f"Resolution : {WIDTH}x{HEIGHT}")
    print(f"FPS        : {FPS}")
    print(f"Destination: {destination_ip}:{port}")
    print("===============================================================")
    # ------------------------------------------------------------------------
    # START UDP/RTP STREAMER
    # ------------------------------------------------------------------------
    streamer = UDPStreamer(destination_ip, port)
    if not streamer.is_ready():
        print("[ERROR] UDP streamer tidak siap")
        run_event.clear()
        return 1

    # ------------------------------------------------------------------------
    # START CAMERA THREAD
    # ------------------------------------------------------------------------
    cam_thread = threading.Thread(
        target=camera_loop,
        name="camera_loop",
        daemon=True,
    )
    cam_thread.start()
    # ------------------------------------------------------------------------
    # WAIT UNTIL SIGNAL
    # ------------------------------------------------------------------------
    try:
        while run_event.is_set():
            time.sleep(0.2)
    except KeyboardInterrupt:
        run_event.clear()
    # ------------------------------------------------------------------------
    # SHUTDOWN
    # ------------------------------------------------------------------------
    run_event.clear()
    cam_thread.join()
    streamer.shutdown()
    print("[INFO] Program selesai.")
    return 0

if __name__ == "__main__":
    sys.exit(main())