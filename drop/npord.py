#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
npord.py
Versi Python dari npord.cpp.

Fungsi utama:
- Membaca MAVLink dari SERIAL_PORT.
- Menunggu HEARTBEAT sebelum memulai sequence.
- Membaca GLOBAL_POSITION_INT, MISSION_CURRENT, HEARTBEAT, VFR_HUD.
- Low-pass filter posisi.
- Estimasi wind dari ground velocity + airspeed + heading.
- Simulasi lintasan payload dengan drag + wind.
- Prediksi titik impact.
- Dynamic impact radius berdasarkan groundspeed dan fall time.
- Membuka pintu payload CH6 pada waypoint + radius yang ditentukan.
- Meng-arm drop setelah WP >= DROP_WP_INDEX.
- Melepas payload CH5 hanya saat predicted impact memenuhi target.
- Menutup CH6 setelah DOOR_CLOSE_DELAY detik.
- Mengirim LAT/LON UAV ke 127.0.0.1:5000 maksimal 10 Hz.

Dependensi:
    pip3 install pymavlink
"""

import math
import os
import sys
import time
import socket
import termios
import select
from dataclasses import dataclass
from typing import Optional, Tuple

from pymavlink import mavutil

# ============================================================================
# PARAMETER KONEKSI & SERVO
# ============================================================================
SERIAL_PORT = "/dev/ttyS7"
BAUDRATE = 115200
# TELEMETRY IPC KE STR VIA UDP LOOPBACK
TELEMETRY_IP = "127.0.0.1"
TELEMETRY_PORT = 5000
TELEMETRY_PUBLISH_INTERVAL = 0.10  # 10 Hz
POSITION_TIMEOUT = 1.0
SERVO_CHANNEL = 5       # Servo Drop Payload
SERVO_DOOR_CHANNEL = 6  # Servo Pintu Payload
PWMD_BUKA = 2000.0 # DROP
PWMD_TUTUP = 1000.0 # DROP
PWMDO_BUKA = 2000.0 # DOOR
PWMDO_TUTUP = 1000.0 # DOOR
SERVO_FEEDBACK_TOLERANCE = 40.0
SERVO_FEEDBACK_TIMEOUT = 1.0

# ============================================================================
# KOORDINAT TARGET PINTU & DROP
# ============================================================================
LAT_DOOR = -0.9310986
LON_DOOR = 116.8483114
LAT_TARGET = -0.9319386
LON_TARGET = 116.8497276 
# WP aktual dibaca dari MISSION_CURRENT.
# WP hanya menjadi mission gate untuk payload.
DOOR_WP_INDEX = 16
DROP_WP_INDEX = 17
DOOR_OPEN_RADIUS = 26.0 # Meter
MIN_DROP_ALTITUDE = 100.0 # Meter

# ============================================================================
# FISIKA & SIMULASI
# ============================================================================
G = 9.81
AIR_DENSITY = 1.225
PAYLOAD_MASS = 0.5
PAYLOAD_AREA = 0.01
DRAG_COEFFICIENT = 1.0
SIMULATION_DT = 0.01
MAX_SIMULATION_TIME = 60.0
SERVO_DELAY = 0.0 # Second
DOOR_CLOSE_DELAY = 15.0 # Second

# ============================================================================
# TIMING & TIMEOUT
# ============================================================================
PRINT_INTERVAL = 0.2
PREDICTION_INTERVAL = 0.10

# ============================================================================
# LPF TAU
# ============================================================================
LPF_TAU_POSITION = 0.20
LPF_TAU_VELOCITY = 0.15   # dipertahankan untuk kesetaraan parameter C++
LPF_TAU_AIRSPEED = 0.15  # dipertahankan untuk kesetaraan parameter C++
LPF_TAU_ALTITUDE = 0.20  # dipertahankan untuk kesetaraan parameter C++

# ============================================================================
# DYNAMIC IMPACT TOLERANCE + RELEASE TRAVEL
# ============================================================================
DYNAMIC_RADIUS_BASE = 3.0
DYNAMIC_RADIUS_MIN = 5.0
DYNAMIC_RADIUS_MAX = 25.0
DYNAMIC_RADIUS_KSIGMA = 2.0
SIGMA_GROUNDSPEED = 1.0
SIGMA_FALL_TIME = 0.30
SIGMA_MODEL_PREDICTION = 5.0

# ============================================================================
# HELPER
# ============================================================================
def monotonic_time() -> float:
    return time.monotonic()

def is_finite(value: float) -> bool:
    return math.isfinite(value)

# ============================================================================
# LOW PASS FILTER
# ============================================================================
class LowPassFilter:
    def __init__(self, tau: float):
        self.tau = tau
        self.value = 0.0
        self.timestamp = 0.0
        self.initialized = False

    def update(self, new_value: float, current_time: float) -> float:
        if not self.initialized:
            self.value = new_value
            self.timestamp = current_time
            self.initialized = True
            return self.value

        dt = current_time - self.timestamp
        self.timestamp = current_time
        dt = max(0.0, min(dt, 1.0))
        alpha = dt / (self.tau + dt) if dt > 0.0 else 0.0
        self.value += alpha * (new_value - self.value)
        return self.value

    def get_value(self) -> float:
        return self.value


# ============================================================================
# WIND ESTIMATOR
# ============================================================================
@dataclass
class WindEstimate:
    east: float = 0.0
    north: float = 0.0
    speed: float = 0.0
    direction: float = 0.0
    valid: bool = False

def estimate_wind_from_airspeed(vx_east: float, vy_north: float, airspeed: float, heading_deg: float) -> WindEstimate:
    result = WindEstimate()
    if not all(map(is_finite, (vx_east, vy_north, airspeed, heading_deg))):
        return result
    
    if airspeed < 0.0 or heading_deg < 0.0 or heading_deg >= 360.0:
        return result

    heading_rad = math.radians(heading_deg)
    v_air_east = airspeed * math.sin(heading_rad)
    v_air_north = airspeed * math.cos(heading_rad)
    result.east = vx_east - v_air_east
    result.north = vy_north - v_air_north
    result.speed = math.hypot(result.east, result.north)
    if result.speed > 1e-6:
        result.direction = math.degrees(math.atan2(result.east, result.north))
        if result.direction < 0.0:
            result.direction += 360.0

    result.valid = True
    return result


# ============================================================================
# DATA STRUCTURES
# ============================================================================
@dataclass
class DroneData:
    lat: float = 0.0
    lon: float = 0.0
    alt_msl: float = 0.0
    alt_agl: float = 0.0
    groundspeed: float = 0.0
    heading: float = 0.0
    vx_east: float = 0.0
    vy_north: float = 0.0
    vz: float = 0.0
    airspeed: float = 0.0
    wind_speed: float = 0.0
    wind_direction: float = 0.0
    wp: int = -1
    mode: str = "UNKNOWN"
    valid_lat: bool = False
    valid_lon: bool = False
    valid_airspeed: bool = False
    valid_heading: bool = False
    valid_wind: bool = False

@dataclass
class ImpactResult:
    x: float
    y: float
    z: float
    time: float
    vx: float
    vy: float
    vz: float

@dataclass
class PredictionResult:
    lat_drop: float
    lon_drop: float
    fall_time: float
    dx: float
    dy: float
    release_lat: float
    release_lon: float
    impact_vx: float
    impact_vy: float
    impact_vz: float

# ============================================================================
# HAVERSINE + METER <-> GPS
# ============================================================================
def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2)
    a = max(0.0, min(1.0, a))
    return 2.0 * r * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

def meter_to_latlon(lat: float, lon: float, dx: float, dy: float) -> Tuple[float, float]:
    dlat = dy / 111320.0
    cos_lat = math.cos(math.radians(lat))
    if abs(cos_lat) < 1e-6:
        cos_lat = 1e-6

    dlon = dx / (111320.0 * cos_lat)
    return lat + dlat, lon + dlon

# ============================================================================
# SIMULASI DRAG + WIND
# ============================================================================
def simulate_payload_impact(alt: float, vx_init: float, vy_init: float, vz_init: float, wind_speed: float, wind_dir: float) -> ImpactResult:
    x = 0.0
    y = 0.0
    z = alt
    vx = vx_init
    vy = vy_init
    vz = vz_init
    theta = math.radians(wind_dir)
    wx = wind_speed * math.sin(theta)
    wy = wind_speed * math.cos(theta)
    t = 0.0
    while z > 0.0 and t < MAX_SIMULATION_TIME:
        vrel_x = vx - wx
        vrel_y = vy - wy
        vrel_z = vz
        vrel = math.sqrt(vrel_x * vrel_x + vrel_y * vrel_y + vrel_z * vrel_z)

        force_drag_x = 0.0
        force_drag_y = 0.0
        force_drag_z = 0.0
        if vrel > 0.0001:
            drag_factor = -0.5 * AIR_DENSITY * DRAG_COEFFICIENT * PAYLOAD_AREA * vrel
            force_drag_x = drag_factor * vrel_x
            force_drag_y = drag_factor * vrel_y
            force_drag_z = drag_factor * vrel_z

        acc_x = force_drag_x / PAYLOAD_MASS
        acc_y = force_drag_y / PAYLOAD_MASS
        acc_z = -G + force_drag_z / PAYLOAD_MASS
        vx += acc_x * SIMULATION_DT
        vy += acc_y * SIMULATION_DT
        vz += acc_z * SIMULATION_DT
        x += vx * SIMULATION_DT
        y += vy * SIMULATION_DT
        z += vz * SIMULATION_DT
        t += SIMULATION_DT
    if z < 0.0:
        z = 0.0

    return ImpactResult(x, y, z, t, vx, vy, vz)

# ============================================================================
# PREDIKSI TITIK JATUH
# ============================================================================
def predict_drop(lat: float, lon: float, alt: float, vx_east: float, vy_north: float, vz: float, wind_speed: float, wind_dir: float) -> PredictionResult:
    dx_release = vx_east * SERVO_DELAY
    dy_release = vy_north * SERVO_DELAY
    alt_release = alt + vz * SERVO_DELAY
    if alt_release <= 0.0:
        alt_release = 0.1

    rel_lat, rel_lon = meter_to_latlon(lat, lon, dx_release, dy_release)
    impact = simulate_payload_impact(alt_release, vx_east, vy_north, vz, wind_speed, wind_dir)
    drop_lat, drop_lon = meter_to_latlon(rel_lat, rel_lon, impact.x, impact.y)
    return PredictionResult(
        lat_drop=drop_lat,
        lon_drop=drop_lon,
        fall_time=impact.time,
        dx=dx_release + impact.x,
        dy=dy_release + impact.y,
        release_lat=rel_lat,
        release_lon=rel_lon,
        impact_vx=impact.vx,
        impact_vy=impact.vy,
        impact_vz=impact.vz
    )

# ============================================================================
# DYNAMIC RADIUS
# ============================================================================
def calculate_release_travel_distance(data: DroneData, fall_time: float) -> float:
    if not is_finite(data.groundspeed) or not is_finite(fall_time):
        return 0.0
    
    if data.groundspeed < 0.0 or fall_time <= 0.0:
        return 0.0
    return data.groundspeed * fall_time

def calculate_dynamic_impact_radius(data: DroneData, fall_time: float) -> float:
    if not is_finite(data.groundspeed) or not is_finite(fall_time):
        return DYNAMIC_RADIUS_MIN
    
    if data.groundspeed < 0.0 or fall_time <= 0.0:
        return DYNAMIC_RADIUS_MIN

    sigma_from_gs = fall_time * SIGMA_GROUNDSPEED
    sigma_from_time = data.groundspeed * SIGMA_FALL_TIME
    sigma_travel = math.sqrt(sigma_from_gs ** 2 + sigma_from_time ** 2)
    sigma_total = math.sqrt(SIGMA_MODEL_PREDICTION ** 2 + sigma_travel ** 2)
    radius = DYNAMIC_RADIUS_BASE + DYNAMIC_RADIUS_KSIGMA * sigma_total
    return max(DYNAMIC_RADIUS_MIN, min(DYNAMIC_RADIUS_MAX, radius))

# ============================================================================
# SERIAL / MAVLINK
# ============================================================================
def open_serial(port: str, baudrate: int) -> int:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty = termios.tcgetattr(fd)
    tty[0] = 0
    tty[1] = 0
    tty[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    tty[3] = 0
    tty[4] = baudrate_constant(baudrate)
    tty[5] = baudrate_constant(baudrate)
    tty[6][termios.VMIN] = 0
    tty[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, tty)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd

def baudrate_constant(baudrate: int) -> int:
    mapping = {
        50: termios.B50,
        75: termios.B75,
        110: termios.B110,
        134: termios.B134,
        150: termios.B150,
        200: termios.B200,
        300: termios.B300,
        600: termios.B600,
        1200: termios.B1200,
        1800: termios.B1800,
        2400: termios.B2400,
        4800: termios.B4800,
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
        460800: termios.B460800,
        500000: getattr(termios, "B500000", termios.B38400),
        576000: getattr(termios, "B576000", termios.B38400),
        921600: getattr(termios, "B921600", termios.B38400),
        1000000: getattr(termios, "B1000000", termios.B38400),
    }
    if baudrate not in mapping:
        raise ValueError(f"Baudrate {baudrate} tidak didukung oleh termios Python ini")
    return mapping[baudrate]

def safe_read(fd: int, size: int = 4096) -> bytes:
    try:
        return os.read(fd, size)
    except BlockingIOError:
        return b""
    except OSError:
        return b""

def send_servo_command(serial_fd: int, mav: mavutil.mavlink.MAVLink, system_id: int, component_id: int, channel: int, pwm: float) -> None:
    # Ekivalen dengan mavlink_msg_command_long_pack() pada C++.
    # Pesan di-encode memakai source system/component 255/190, kemudian
    # hasil packet benar-benar ditulis langsung ke file descriptor serial.
    message = mav.command_long_encode(
        system_id,
        component_id,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        float(channel),
        float(pwm),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    packet = message.pack(mav)
    total = 0
    while total < len(packet):
        try:
            written = os.write(serial_fd, packet[total:])
        except BlockingIOError:
            time.sleep(0.001)
            continue
        if written <= 0:
            raise OSError("Gagal menulis packet MAVLink COMMAND_LONG ke serial")
        total += written


def create_telemetry_udp_socket() -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    print(f"[UDP] Telemetry -> {TELEMETRY_IP}:{TELEMETRY_PORT}")
    return sock

def publish_telemetry(sock: socket.socket, data: DroneData) -> None:
    if not data.valid_lat or not data.valid_lon:
        return

    packet = f"LAT={data.lat:.8f} LON={data.lon:.8f}".encode("ascii")
    try:
        sock.sendto(packet, (TELEMETRY_IP, TELEMETRY_PORT))
    except OSError as exc:
        if not getattr(publish_telemetry, "warned", False):
            print(f"[UDP] Gagal mengirim telemetry ke str: {exc}", file=sys.stderr)
            publish_telemetry.warned = True

def configure_mavlink_parser() -> mavutil.mavfile:
    """
    Dummy mavfile hanya digunakan untuk akses mav.mav.command_long_send dan dialect.
    Parsing byte tetap dilakukan dengan mavutil.mavlink.MAVLink parser langsung.
    """
    # device tidak dibuka oleh mavutil.serial untuk menghindari double-open pada ttyS7.
    # MAVLink signing/serial stream tidak digunakan di program asli.
    mav = mavutil.mavlink_connection("udpin:127.0.0.1:0", dialect="common")
    return mav

def main() -> int:
    print("Connecting to Pixhawk...")
    serial_fd: Optional[int] = None
    telemetry_sock: Optional[socket.socket] = None
    try:
        try:
            serial_fd = open_serial(SERIAL_PORT, BAUDRATE)
        except Exception as exc:
            print(f"open({SERIAL_PORT}) gagal: {exc}", file=sys.stderr)
            print("Gagal membuka port serial!", file=sys.stderr)
            return 1
        # Parser MAVLink raw stream, set source ID agar mirip command_long_pack(255,190,...)
        parser = mavutil.mavlink.MAVLink(None)
        parser.srcSystem = 255
        parser.srcComponent = 190
        # Object parser juga digunakan untuk encode COMMAND_LONG; tidak membuka serial lagi.
        mav_sender = parser
        system_id = 1
        component_id = 1
        # --------------------------------------------------------------------
        # WAIT HEARTBEAT
        # --------------------------------------------------------------------
        connected = False
        start_time = monotonic_time()
        while not connected:
            raw = safe_read(serial_fd, 4096)
            if raw:
                for byte in raw:
                    try:
                        msg = parser.parse_char(bytes((byte,)))
                    except Exception:
                        msg = None

                    if msg is not None and msg.get_msgId() == mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT:
                        print(" <-- HEARTBEAT")
                        system_id = msg.get_srcSystem()
                        component_id = msg.get_srcComponent()
                        connected = True
                        break

            if monotonic_time() - start_time > 10.0:
                print("\nERROR: Tidak menerima HEARTBEAT dari Pixhawk selama 10 detik.", file=sys.stderr)
                return 1
            
            time.sleep(0.01)
        print("Connected to Pixhawk!")
        try:
            telemetry_sock = create_telemetry_udp_socket()
        except OSError as exc:
            print(f"Peringatan: telemetry UDP tidak aktif; npord tetap berjalan. {exc}")
        lat_filter = LowPassFilter(LPF_TAU_POSITION)
        lon_filter = LowPassFilter(LPF_TAU_POSITION)
        data = DroneData()
        drop_done = False
        door_opened = False
        drop_gate_armed = False
        door_close_pending = False
        door_close_time = 0.0
        last_print_time = 0.0
        last_prediction_time = 0.0
        last_udp_publish_time = 0.0
        last_position_update_time = 0.0
        # INIT TUTUP SERVO
        send_servo_command(serial_fd, mav_sender, system_id, component_id, SERVO_DOOR_CHANNEL, PWMDO_TUTUP)
        send_servo_command(serial_fd, mav_sender, system_id, component_id, SERVO_CHANNEL, PWMD_TUTUP)
        while True:
            raw = safe_read(serial_fd, 4096)
            current_time = monotonic_time()
            if raw:
                for byte in raw:
                    try:
                        msg = parser.parse_char(bytes((byte,)))
                    except Exception:
                        msg = None
                    if msg is None:
                        continue

                    msg_id = msg.get_msgId()
                    # --------------------------------------------------------
                    # GLOBAL_POSITION_INT
                    # --------------------------------------------------------
                    if msg_id == mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                        data.lat = lat_filter.update(msg.lat / 1e7, current_time)
                        data.lon = lon_filter.update(msg.lon / 1e7, current_time)
                        data.alt_agl = msg.relative_alt / 1000.0
                        data.vx_east = msg.vx / 100.0
                        data.vy_north = msg.vy / 100.0
                        data.vz = -msg.vz / 100.0
                        data.groundspeed = math.hypot(data.vx_east, data.vy_north)
                        data.valid_lat = True
                        data.valid_lon = True
                        last_position_update_time = current_time
                    # --------------------------------------------------------
                    # MISSION_CURRENT
                    # --------------------------------------------------------
                    elif msg_id == mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT:
                        data.wp = int(msg.seq)
                    # --------------------------------------------------------
                    # HEARTBEAT
                    # --------------------------------------------------------
                    elif msg_id == mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT:
                        data.mode = "AUTO" if msg.custom_mode == 10 else "OTHER"
                    # --------------------------------------------------------
                    # VFR_HUD
                    # --------------------------------------------------------
                    elif msg_id == mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD:
                        data.airspeed = float(msg.airspeed)
                        data.groundspeed = float(msg.groundspeed)
                        data.heading = float(msg.heading)
                        data.valid_airspeed = is_finite(data.airspeed) and data.airspeed >= 0.0
                        data.valid_heading = is_finite(data.heading) and 0.0 <= data.heading < 360.0
            else:
                # C++ memberi delay 5 ms jika tidak ada byte baru.
                time.sleep(0.005)

            # ----------------------------------------------------------------
            # UDP TELEMETRY, maksimum 10 Hz, hanya posisi fresh
            # ----------------------------------------------------------------
            position_fresh = (data.valid_lat and data.valid_lon and last_position_update_time > 0.0 and (current_time - last_position_update_time <= POSITION_TIMEOUT))
            if (telemetry_sock is not None and position_fresh and (current_time - last_udp_publish_time >= TELEMETRY_PUBLISH_INTERVAL)):
                publish_telemetry(telemetry_sock, data)
                last_udp_publish_time = current_time

            # ----------------------------------------------------------------
            # Untuk logika payload, posisi harus fresh
            # ----------------------------------------------------------------
            if not position_fresh:
                print("Menunggu Sinyal GPS/posisi realtime...   ", end="\r", flush=True)
                continue

            if current_time - last_prediction_time >= PREDICTION_INTERVAL:
                last_prediction_time = current_time
                # 1. Estimasi wind
                wind = estimate_wind_from_airspeed(data.vx_east, data.vy_north, data.airspeed, data.heading)
                data.valid_wind = data.valid_airspeed and data.valid_heading and wind.valid
                wind_speed = wind.speed if data.valid_wind else 0.0
                wind_direction = wind.direction if data.valid_wind else 0.0
                # 2. Prediksi impact
                pred = predict_drop(data.lat, data.lon, data.alt_agl, data.vx_east, data.vy_north, data.vz, wind_speed, wind_direction)
                dynamic_drop_travel = calculate_release_travel_distance(data, pred.fall_time)
                dynamic_impact_radius = calculate_dynamic_impact_radius(data, pred.fall_time)
                # 3. Jarak
                dist_to_door = haversine(data.lat, data.lon, LAT_DOOR, LON_DOOR)
                raw_dist_to_drop = haversine(data.lat, data.lon, LAT_TARGET, LON_TARGET)
                pred_dist_to_drop = haversine(pred.lat_drop, pred.lon_drop, LAT_TARGET, LON_TARGET)
                # ----------------------------------------------------------------
                # PRINT STATUS
                # ----------------------------------------------------------------
                if current_time - last_print_time >= PRINT_INTERVAL:
                    print("\n========================================")
                    print(f"Posisi   : {data.lat:.7f}, {data.lon:.7f}")
                    print(f"WP       : {data.wp}\t | Mode : {data.mode}")
                    print(
                        f"Alt Rel  : {data.alt_agl:.2f} m\t | "
                        f"Airspeed : {data.airspeed:.2f} m/s\t | "
                        f"Heading : {data.heading:.2f} deg"
                    )
                    if data.valid_wind:
                        print(
                            f"Wind Est : {wind.speed:.2f} m/s\t | "
                            f"Der : {wind.direction:.2f} deg"
                        )
                    else:
                        print("Wind Est : INVALID (airspeed/heading belum valid)")

                    print(f"Jarak ke Titik Pintu : {dist_to_door:.2f} m")
                    print(f"Jarak GPS ke Target  : {raw_dist_to_drop:.2f} m")
                    print(f"Jarak Prediksi Target: {pred_dist_to_drop:.2f} m")
                    print(f"Fall Time             : {pred.fall_time:.2f} s")
                    print(f"Jarak Tempuh Jatuh    : {dynamic_drop_travel:.2f} m")
                    print(f"Radius Impact Dinamis : {dynamic_impact_radius:.2f} m")
                    print(f"Drop Gate             : {'ARMED' if drop_gate_armed else 'WAIT'}")
                    print(
                        f"Status   : Door = {'OPEN' if door_opened else 'CLOSED'} "
                        f"| Drop = {'DONE' if drop_done else 'WAIT'}"
                    )
                    print("========================================")
                    last_print_time = current_time

                # ----------------------------------------------------------------
                # A. KONDISI BUKA PINTU
                # ----------------------------------------------------------------
                waypointdoor_ok = data.wp == DOOR_WP_INDEX
                door_condition = (dist_to_door <= DOOR_OPEN_RADIUS and waypointdoor_ok and not door_opened)
                if door_condition:
                    print("\n=== KOORDINAT PINTU TERCAPAI! MEMBUKA PINTU (CH 6) ===")
                    send_servo_command(serial_fd, mav_sender, system_id, component_id, SERVO_DOOR_CHANNEL, PWMDO_BUKA)
                    door_opened = True

                # ----------------------------------------------------------------
                # B. WP DROP = MISSION GATE
                # ----------------------------------------------------------------
                if not drop_gate_armed and data.wp >= DROP_WP_INDEX:
                    drop_gate_armed = True
                    print("\n=== DROP MISSION GATE ARMED ===")
                    print(f"WP Gate         : {data.wp}")
                    print(f"Target Asli     : {LAT_TARGET}, {LON_TARGET}")
                    print("CH5 Trigger     : PREDICTED IMPACT")

                # ----------------------------------------------------------------
                # C. KONDISI DROP PAYLOAD
                # ----------------------------------------------------------------
                prediction_ok = (dynamic_impact_radius > 0.0 and is_finite(pred_dist_to_drop) and pred_dist_to_drop <= dynamic_impact_radius)
                altitude_ok = (is_finite(data.alt_agl) and data.alt_agl >= MIN_DROP_ALTITUDE)
                mode_ok = data.mode == "AUTO"
                telemetry_ok = (data.valid_airspeed and data.valid_heading and is_finite(data.groundspeed) and is_finite(data.vx_east) and is_finite(data.vy_north) and is_finite(data.vz))
                target_condition = (drop_gate_armed and prediction_ok and telemetry_ok and mode_ok and altitude_ok and not drop_done)
                if target_condition:
                    print("\n=== PREDICTED IMPACT MEMENUHI TARGET ===")
                    print(f"WP Gate         : {data.wp} (sudah ARM)")
                    print(f"Jarak GPS Target: {raw_dist_to_drop:.2f} m")
                    print(f"Predicted Impact: {pred_dist_to_drop:.2f} m")
                    print(f"Radius Impact   : {dynamic_impact_radius:.2f} m")
                    print(f"Jarak Tempuh    : {dynamic_drop_travel:.2f} m")
                    print(f"Fall Time       : {pred.fall_time:.2f} s")
                    print(f"Altitude AGL    : {data.alt_agl:.2f} m")
                    print(f"Airspeed        : {data.airspeed:.2f} m/s")
                    print(f"Groundspeed     : {data.groundspeed:.2f} m/s")
                    print(f"Mode            : {data.mode}")
                    print("=== MELEPAS PAYLOAD (CH 5) ===")
                    send_servo_command(serial_fd, mav_sender, system_id, component_id, SERVO_CHANNEL, PWMD_BUKA)
                    drop_done = True

                    # TIMER PENUTUPAN CH6
                    door_close_pending = True
                    door_close_time = current_time + DOOR_CLOSE_DELAY
                    print(
                        f"=== PINTU CH6 AKAN DITUTUP DALAM "
                        f"{DOOR_CLOSE_DELAY:.0f} DETIK ==="
                    )

                # ----------------------------------------------------------------
                # AUTO CLOSE PINTU PAYLOAD SETELAH DROP
                # ----------------------------------------------------------------
                if door_close_pending and current_time >= door_close_time:
                    send_servo_command(serial_fd, mav_sender, system_id, component_id, SERVO_DOOR_CHANNEL, PWMDO_TUTUP,)
                    door_opened = False
                    door_close_pending = False
                    print("\n=== DELAY SELESAI ===")
                    print("=== PINTU PAYLOAD CH6 DITUTUP KEMBALI ===")
                    if drop_done and not door_opened and not door_close_pending:
                        print("\n========================================")
                        print("=== SEQUENCE PAYLOAD SELESAI          ===")
                        print("=== CH5 : PAYLOAD SUDAH DROP          ===")
                        print("=== CH6 : PINTU SUDAH DITUTUP         ===")
                        print("=== PROGRAM BERHENTI                   ===")
                        print("========================================")
                        return 0

    except KeyboardInterrupt:
        print("\nProgram dihentikan oleh user (Ctrl+C).")
        return 130
    except Exception as exc:
        print(f"\n[FATAL] {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        if telemetry_sock is not None:
            try:
                telemetry_sock.close()
            except Exception:
                pass
        if serial_fd is not None:
            try:
                os.close(serial_fd)
            except Exception:
                pass

if __name__ == "__main__":
    raise SystemExit(main())