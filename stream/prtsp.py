#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
crtsp.py
Versi Python dari crtsp.cpp.

Fungsi:
- Streaming kamera melalui RTSP menggunakan GStreamer.
- Kamera OpenCV/V4L2, auto-detect /dev/video0 .. /dev/video9.
- Resolusi 640x360 @ 30 FPS.
- Recording MJPG ke record/Nasional_YYYYMMDD_HHMMSS.mkv.
- Buffer frame 1 detik.
- Rotasi file recording setiap 1 jam.
- Recovery jika kamera tidak ditemukan, frame kosong, atau kamera freeze.

Usage:
    python3 crtsp.py <IP_ADDRESS>

Example:
    python3 crtsp.py 0.0.0.0

RTSP:
    rtsp://<IP_ADDRESS>:8554/live
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
try:
    import gi
    gi.require_version("Gst", "1.0")
    gi.require_version("GstApp", "1.0")
    gi.require_version("GstRtspServer", "1.0")
    from gi.repository import Gst, GstApp, GstRtspServer, GLib
except ImportError as exc:
    print("[ERROR] Modul PyGObject/GStreamer tidak tersedia.")
    print(f"[DETAIL] {exc}")
    print(
        "Install paket yang diperlukan, misalnya:\n"
        "  sudo apt install python3-gi "
        "gir1.2-gstreamer-1.0 "
        "gir1.2-gst-rtsp-server-1.0"
    )
    sys.exit(1)

# ============================================================================
# KONFIGURASI
# ============================================================================
WIDTH = 640
HEIGHT = 360
FPS = 30
BUFF_SEC = 1
ROTATE_SEC = 3600
CAMERA_SCAN_COUNT = 10
RTSP_PORT = 8554
RTSP_MOUNT = "/live"
RECONNECT_DELAY_SEC = 0.5
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
main_loop = None
global_appsrc = None
global_appsrc_lock = threading.Lock()
frame_counter = 0
frame_counter_lock = threading.Lock()

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
# SIGNAL / SHUTDOWN
# ============================================================================
def request_shutdown(signum=None, frame=None):
    global main_loop
    print("\n[INFO] Shutdown...")
    run_event.clear()
    if main_loop is not None:
        try:
            main_loop.quit()
        except Exception:
            pass

# ============================================================================
# RECORDING
# ============================================================================
def create_writer(name: str):
    RECORD_DIR.mkdir(parents=True, exist_ok=True)
    now = datetime.now()
    filename = RECORD_DIR / f"{name}_{now:%Y%m%d_%H%M%S}.mkv"
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    writer = cv2.VideoWriter(str(filename), fourcc, FPS, (WIDTH, HEIGHT))
    if not writer.isOpened():
        print(f"[ERROR] Writer gagal: {filename}")
    else:
        print(f"[INFO] Recording: {filename}")
    return writer, time.time()

# ============================================================================
# CAMERA
# ============================================================================
def device_exists(index: int) -> bool:
    path = f"/dev/video{index}"
    return os.path.exists(path) and os.path.isfile(path)

def open_camera():
    for index in range(CAMERA_SCAN_COUNT):
        if not device_exists(index):
            continue

        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue

        print(f"[INFO] Kamera ditemukan di /dev/video{index}")
        # Sama seperti crtsp.cpp.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, FPS)
        return cap, index
    return None, -1

def reset_uvcvideo():
    """
    Meniru recovery driver pada crtsp.cpp.
    Memerlukan sudo/NOPASSWD jika proses non-root.
    """
    print("[RECOVERY] Reset driver kamera")
    # Jangan membuat thread berhenti hanya karena modprobe gagal.
    commands = ["sudo modprobe -r uvcvideo", "sudo modprobe uvcvideo"]
    for command in commands:
        rc = os.system(command)
        if rc != 0:
            print(f"[WARN] Perintah gagal (rc={rc}): {command}")
    time.sleep(0.5)

# ============================================================================
# GSTREAMER APP-SRC
# ============================================================================
def get_appsrc():
    with global_appsrc_lock:
        return global_appsrc

def reset_appsrc():
    global global_appsrc
    with global_appsrc_lock:
        old = global_appsrc
        global_appsrc = None

    if old is not None:
        try:
            old.set_state(Gst.State.NULL)
        except Exception:
            pass

def push_frame_to_appsrc(frame):
    global frame_counter
    appsrc = get_appsrc()
    if appsrc is None:
        return

    if not frame.flags["C_CONTIGUOUS"]:
        frame = frame.copy()

    data = frame.tobytes()
    try:
        gst_buffer = Gst.Buffer.new_allocate(None, len(data), None)
        if gst_buffer is None:
            print("[GSTREAMER ERROR] Gagal membuat GstBuffer")
            return

        success, map_info = gst_buffer.map(Gst.MapFlags.WRITE)
        if not success:
            print("[GSTREAMER ERROR] Gagal map GstBuffer")
            gst_buffer.unref()
            return
        
        try:
            map_info.data[:len(data)] = data
        finally:
            gst_buffer.unmap(map_info)
        with frame_counter_lock:
            pts = frame_counter * Gst.SECOND // FPS
            frame_counter += 1
        gst_buffer.pts = pts
        gst_buffer.dts = pts
        gst_buffer.duration = Gst.SECOND // FPS
        ret = appsrc.emit("push-buffer", gst_buffer)
        if ret != Gst.FlowReturn.OK:
            print(f"[GSTREAMER] push-buffer: {ret}")
    except Exception as exc:
        print(f"[GSTREAMER ERROR] push-buffer gagal: {exc}")

# ============================================================================
# CAMERA LOOP
# ============================================================================
def camera_loop():
    global frame_counter
    cap = None
    cam_index = -1
    fail_count = 0
    no_frame_count = 0
    last_frame_time = time.monotonic()
    frame_buffer = deque(maxlen=FPS * BUFF_SEC)
    fwriter, ftime = create_writer(RECORD_PREFIX)
    try:
        while run_event.is_set():
            # ----------------------------------------------------------------
            # OPEN / REOPEN CAMERA
            # ----------------------------------------------------------------
            if cap is None or not cap.isOpened():
                cap, cam_index = open_camera()
                if cap is None or not cap.isOpened():
                    fail_count += 1
                    print(f"[WARN] Tidak ada kamera ({fail_count})")
                    if fail_count > DRIVER_RESET_AFTER:
                        reset_uvcvideo()
                        fail_count = 0

                    time.sleep(RECONNECT_DELAY_SEC)
                    continue
                fail_count = 0
                no_frame_count = 0
                last_frame_time = time.monotonic()
            # ----------------------------------------------------------------
            # READ FRAME
            # ----------------------------------------------------------------
            ok, frame = cap.read()
            if not ok or frame is None or frame.size == 0:
                no_frame_count += 1
                print(f"[WARN] Frame kosong ({no_frame_count}/10)")
                if no_frame_count > NO_FRAME_LIMIT:
                    print("[ERROR] Kamera hang -> reconnect")
                    cap.release()
                    cap = None
                    no_frame_count = 0
                    time.sleep(RECONNECT_DELAY_SEC)
                continue

            no_frame_count = 0
            # ----------------------------------------------------------------
            # FREEZE CHECK
            # ----------------------------------------------------------------
            now = time.monotonic()
            if now - last_frame_time > CAMERA_FREEZE_SEC:
                print("[ERROR] Kamera freeze -> reconnect paksa")
                cap.release()
                cap = None
                time.sleep(RECONNECT_DELAY_SEC)
                continue

            last_frame_time = now
            # ----------------------------------------------------------------
            # FPS
            # ----------------------------------------------------------------
            fps.update()
            # ----------------------------------------------------------------
            # RECORDING
            # ----------------------------------------------------------------
            if fwriter is not None and fwriter.isOpened():
                try:
                    fwriter.write(frame)
                except Exception as exc:
                    print(f"[ERROR] Gagal menulis recording: {exc}")
            # ----------------------------------------------------------------
            # BUFFER
            # ----------------------------------------------------------------
            try:
                frame_buffer.append(frame.copy())
            except Exception as exc:
                print(f"[WARN] Gagal buffer frame: {exc}")
            # ----------------------------------------------------------------
            # ROTATE RECORDING
            # ----------------------------------------------------------------
            if time.time() - ftime > ROTATE_SEC:
                if fwriter is not None:
                    fwriter.release()

                fwriter, ftime = create_writer(RECORD_PREFIX)
            # ----------------------------------------------------------------
            # RTSP APP-SRC
            # ----------------------------------------------------------------
            push_frame_to_appsrc(frame)
    except Exception as exc:
        print(f"[ERROR] camera_loop: {exc}")
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

# ============================================================================
# RTSP MEDIA CONFIGURE
# ============================================================================
def on_media_configure(factory, media):
    """
    Dipanggil ketika client membuat/akses RTSP media.
    Mengambil appsrc bernama 'mysrc' dari pipeline RTSP.
    """
    global global_appsrc, frame_counter
    try:
        element = media.get_element()
        appsrc = element.get_by_name("mysrc")
        if appsrc is None:
            print("[ERROR] appsrc 'mysrc' tidak ditemukan")
            return

        # Lepaskan referensi lama secara aman.
        with global_appsrc_lock:
            global_appsrc = appsrc
        with frame_counter_lock:
            frame_counter = 0
        try:
            appsrc.set_property("stream-type", GstApp.AppStreamType.STREAM)
        except Exception:
            pass
        try:
            appsrc.set_property("format", Gst.Format.TIME)
        except Exception:
            pass
        try:
            appsrc.set_property("is-live", True)
        except Exception:
            pass
        try:
            appsrc.set_property("do-timestamp", True)
        except Exception:
            pass
        try:
            appsrc.set_property("block", True)
        except Exception:
            pass
        print("[RTSP] Client/media configured")
    except Exception as exc:
        print(f"[RTSP ERROR] media-configure: {exc}")

# ============================================================================
# RTSP SERVER
# ============================================================================
class RTSPServer:
    def __init__(self, port: int = RTSP_PORT):
        self.server = GstRtspServer.RTSPServer.new()
        if self.server is None:
            raise RuntimeError("Gagal membuat RTSP server")

        self.server.set_service(str(port))
        mounts = self.server.get_mount_points()
        factory = GstRtspServer.RTSPMediaFactory.new()

        launch = (
            f"( appsrc name=mysrc is-live=true "
            f"format=time do-timestamp=true block=true "
            f"caps=video/x-raw,format=BGR,width={WIDTH},height={HEIGHT},framerate={FPS}/1 "
            f"! queue leaky=downstream max-size-buffers=1 "
            f"! videoconvert "
            f"! video/x-raw,format=NV12 "
            f"! x264enc tune=zerolatency speed-preset=ultrafast "
            f"bitrate=1200 key-int-max=15 bframes=0 "
            f"! h264parse config-interval=1 "
            f"! rtph264pay pt=96 name=pay0 config-interval=1 )"
        )

        factory.set_launch(launch)
        factory.set_shared(True)
        mounts.add_factory(RTSP_MOUNT, factory)
        self.server.connect("client-connected", self._on_client_connected)
        # media-configure berada pada factory, seperti crtsp.cpp.
        factory.connect("media-configure", on_media_configure)
        self.factory = factory
        self.mounts = mounts

    @staticmethod
    def _on_client_connected(server, client):
        print("[RTSP] Client connected")

    def run(self, ip: str):
        self.server.set_address(ip)
        attach_id = self.server.attach(None)
        if attach_id == 0:
            raise RuntimeError("Gagal attach RTSP server")

        print("\n[INFO] RTSP Stream Ready")
        print(f"rtsp://{ip}:{RTSP_PORT}{RTSP_MOUNT}")
        print(f"[INFO] Resolution : {WIDTH}x{HEIGHT} @ {FPS} FPS")
        print("[INFO] Codec      : H.264 x264 ultrafast, zerolatency")
        return True

def main() -> int:
    global main_loop
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <IP_ADDRESS>")
        return 1

    ip_address = sys.argv[1]
    try:
        Gst.init(None)
        signal.signal(signal.SIGINT, request_shutdown)
        signal.signal(signal.SIGTERM, request_shutdown)
        # RTSP server harus siap lebih dahulu.
        server = RTSPServer(RTSP_PORT)
        server.run(ip_address)
        # Kamera berjalan pada thread terpisah seperti crtsp.cpp.
        camera_thread = threading.Thread(
            target=camera_loop,
            name="camera_loop",
            daemon=True,
        )
        camera_thread.start()
        main_loop = GLib.MainLoop()
        while run_event.is_set():
            # GLib MainLoop tetap perlu berjalan agar RTSP server aktif.
            context = main_loop.get_context()
            while context.pending():
                context.iteration(False)
            time.sleep(0.02)
        camera_thread.join(timeout=3.0)
        reset_appsrc()
        print("[INFO] DONE")
        return 0
    except KeyboardInterrupt:
        request_shutdown()
        return 0
    except Exception as exc:
        print(f"[FATAL] {exc}")
        return 1

if __name__ == "__main__":
    sys.exit(main())