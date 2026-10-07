#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
NPORD - Python version
Port dari drop.cpp / NPORD versi tanpa sensor airspeed.

Fungsi utama:
- Membaca MAVLink dari /dev/ttyS7 @ 115200
- Menunggu HEARTBEAT maksimal 10 detik
- Membaca GLOBAL_POSITION_INT untuk:
    * latitude / longitude
    * relative altitude
    * vx / vy / vz
    * groundspeed
    * ground heading
- Membaca MISSION_CURRENT untuk WP
- Membaca HEARTBEAT untuk mode AUTO
- Tidak menggunakan VFR_HUD / sensor airspeed
- Menggunakan FIXED_WIND_SPEED + FIXED_WIND_DIRECTION sebagai asumsi angin
- Simulasi payload dengan gravitasi + drag + wind
- Mengirim telemetry "LAT=... LON=..." ke 127.0.0.1:5000
- CH6 membuka pintu pada WP 16 dan radius 26 m
- WP >= 17 meng-arm drop gate
- CH5 drop hanya jika predicted impact masuk dynamic impact radius,
  altitude cukup, mode AUTO, dan velocity GPS valid
- CH6 ditutup 15 detik setelah drop

Catatan:
Untuk kompatibilitas dengan drop.cpp, pemetaan velocity dipertahankan:
    data.vx_east  <- MAVLink GLOBAL_POSITION_INT.vx / 100
    data.vy_north <- MAVLink GLOBAL_POSITION_INT.vy / 100
Walaupun definisi MAVLink umum untuk vx/vy perlu diperhatikan saat validasi
lapangan. Jangan mengubah mapping ini tanpa menguji terhadap sistem Anda.
"""

import math
import socket
import sys
import time
from dataclasses import dataclass

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink belum terpasang.")
    print("Instal dengan:")
    print("    python3 -m pip install pymavlink")
    sys.exit(1)

# ============================================================================
# PARAMETER KONEKSI & SERVO
# ============================================================================
SERIAL_PORT = "/dev/ttyS7"
BAUDRATE = 115200
TELEMETRY_IP = "127.0.0.1"
TELEMETRY_PORT = 5000
TELEMETRY_PUBLISH_INTERVAL = 0.10
POSITION_TIMEOUT = 1.0
SERVO_CHANNEL = 5
SERVO_DOOR_CHANNEL = 6
PWM_BUKA = 2000
PWM_TUTUP = 1000

# ============================================================================
# KOORDINAT TARGET PINTU & DROP
# ============================================================================
LAT_DOOR = -0.9310986
LON_DOOR = 116.8483114
LAT_TARGET = -0.9319386
LON_TARGET = 116.8497276

# ============================================================================
# WP / DROP GATE
# ============================================================================
DOOR_WP_INDEX = 16
DROP_WP_INDEX = 17
DOOR_OPEN_RADIUS = 26.0
MIN_DROP_ALTITUDE = 100.0

# ============================================================================
# ASUMSI ANGIN MANUAL - PENGGANTI SENSOR AIRSPEED
#
# wind_direction:
#   0   = North
#   90  = East
#   180 = South
#   270 = West
# Ini adalah arah GERAK angin (toward).
# ============================================================================
FIXED_WIND_SPEED = 0.0
FIXED_WIND_DIRECTION = 0.0

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
SERVO_DELAY = 0.0
DOOR_CLOSE_DELAY = 15.0

# ============================================================================
# TIMING
# ============================================================================
PRINT_INTERVAL = 0.2
PREDICTION_INTERVAL = 0.10

# ============================================================================
# LOW PASS FILTER
# ============================================================================
LPF_TAU_POSITION = 0.20
LPF_TAU_VELOCITY = 0.15

# ============================================================================
# DYNAMIC IMPACT RADIUS
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
def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))

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
        dt = clamp(dt, 0.0, 1.0)
        alpha = dt / (self.tau + dt) if dt > 0.0 else 0.0
        self.value += alpha * (new_value - self.value)
        return self.value

# ============================================================================
# DATA DRONE
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
    wp: int = -1
    mode: str = "UNKNOWN"
    valid_lat: bool = False
    valid_lon: bool = False
    valid_velocity: bool = False

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
# HAVERSINE
# ============================================================================
def haversine(lat1, lon1, lat2, lon2):
    radius_earth = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2)
    a = clamp(a, 0.0, 1.0)
    return 2.0 * radius_earth * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

# ============================================================================
# KONVERSI METER KE GPS
# dx = East (+)
# dy = North (+)
# ============================================================================
def meter_to_latlon(lat, lon, dx, dy):
    dlat = dy / 111320.0
    cos_lat = math.cos(math.radians(lat))
    if abs(cos_lat) < 1e-6:
        cos_lat = 1e-6

    dlon = dx / (111320.0 * cos_lat)
    return lat + dlat, lon + dlon

# ============================================================================
# SIMULASI DRAG + WIND
# ============================================================================
def simulate_payload_impact(alt, vx_init, vy_init, vz_init, wind_speed, wind_dir):
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
            drag_factor = (-0.5 * AIR_DENSITY * DRAG_COEFFICIENT * PAYLOAD_AREA * vrel)
            force_drag_x = drag_factor * vrel_x
            force_drag_y = drag_factor * vrel_y
            force_drag_z = drag_factor * vrel_z

        acc_x = force_drag_x / PAYLOAD_MASS
        acc_y = force_drag_y / PAYLOAD_MASS
        acc_z = -G + (force_drag_z / PAYLOAD_MASS)
        vx += acc_x * SIMULATION_DT
        vy += acc_y * SIMULATION_DT
        vz += acc_z * SIMULATION_DT
        x += vx * SIMULATION_DT
        y += vy * SIMULATION_DT
        z += vz * SIMULATION_DT
        t += SIMULATION_DT
    if z < 0.0:
        z = 0.0

    return ImpactResult(x=x, y=y, z=z, time=t, vx=vx, vy=vy, vz=vz)

# ============================================================================
# PREDIKSI TITIK JATUH
# ============================================================================
def predict_drop(lat, lon, alt, vx_east, vy_north, vz, wind_speed, wind_dir):
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
# DYNAMIC IMPACT RADIUS
# ============================================================================
def calculate_release_travel_distance(data: DroneData, fall_time: float):
    if (not math.isfinite(data.groundspeed) or not math.isfinite(fall_time) or data.groundspeed < 0.0 or fall_time <= 0.0):
        return 0.0

    return data.groundspeed * fall_time

def calculate_dynamic_impact_radius(data: DroneData, fall_time: float):
    if (not math.isfinite(data.groundspeed) or not math.isfinite(fall_time) or data.groundspeed < 0.0 or fall_time <= 0.0):
        return DYNAMIC_RADIUS_MIN

    sigma_from_gs = fall_time * SIGMA_GROUNDSPEED
    sigma_from_time = data.groundspeed * SIGMA_FALL_TIME
    sigma_travel = math.sqrt(sigma_from_gs * sigma_from_gs + sigma_from_time * sigma_from_time)
    sigma_total = math.sqrt(SIGMA_MODEL_PREDICTION * SIGMA_MODEL_PREDICTION + sigma_travel * sigma_travel)
    radius = DYNAMIC_RADIUS_BASE + (DYNAMIC_RADIUS_KSIGMA * sigma_total)
    return clamp(radius, DYNAMIC_RADIUS_MIN, DYNAMIC_RADIUS_MAX)

# ============================================================================
# UDP TELEMETRY -> STR
# ============================================================================
def create_telemetry_socket():
    try:
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        destination = (TELEMETRY_IP, TELEMETRY_PORT)
        print(f"[UDP] Telemetry -> " f"{TELEMETRY_IP}:{TELEMETRY_PORT}")
        return udp_socket, destination
    except OSError as exc:
        print(f"UDP telemetry socket gagal: {exc}", file=sys.stderr)
        return None, None

def publish_telemetry(udp_socket, destination, data: DroneData):
    if (udp_socket is None or destination is None or not data.valid_lat or not data.valid_lon):
        return

    packet = (f"LAT={data.lat:.8f} " f"LON={data.lon:.8f}").encode("ascii")
    try:
        udp_socket.sendto(packet, destination)
    except OSError as exc:
        # Receiver boleh belum hidup; npord tetap berjalan.
        if not getattr(publish_telemetry, "_warned", False):
            print(f"[UDP] Gagal mengirim telemetry ke str: {exc}", file=sys.stderr)
            publish_telemetry._warned = True

# ============================================================================
# MAVLINK
# ============================================================================
def connect_pixhawk():
    print("============================================================")
    print("     NPORD - TANPA SENSOR AIRSPEED - PYTHON")
    print("============================================================")
    print(
        f"Wind asumsi : " f"{FIXED_WIND_SPEED:.2f} m/s @ "
        f"{FIXED_WIND_DIRECTION:.1f} deg"
    )
    print(f"Serial      : {SERIAL_PORT} @ {BAUDRATE}")
    print("============================================================")
    print("Connecting to Pixhawk...")
    try:
        master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUDRATE, autoreconnect=True)
    except Exception as exc:
        print(f"Gagal membuka {SERIAL_PORT}: {exc}", file=sys.stderr)
        return None
    try:
        heartbeat = master.wait_heartbeat(timeout=10.0)
    except Exception:
        heartbeat = None
    if heartbeat is None:
        print(
            "\nERROR: Tidak menerima HEARTBEAT "
            "dari Pixhawk selama 10 detik.",
            file=sys.stderr)
        try:
            master.close()
        except Exception:
            pass
        return None

    print(" <-- HEARTBEAT "
        f"SYSID={master.target_system} "
        f"COMPID={master.target_component}"
    )
    print("Connected to Pixhawk!")
    return master

# ============================================================================
# SERVO COMMAND
# ============================================================================
def send_servo_command(master, system_id, component_id, channel, pwm):
    try:
        master.mav.command_long_send(
            int(system_id),
            int(component_id),
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
        print(f"[SERVO] CH{channel} -> PWM {pwm}")
    except Exception as exc:
        print(
            f"[SERVO] Gagal mengirim CH{channel} "
            f"PWM {pwm}: {exc}", file=sys.stderr)

# ============================================================================
# UPDATE MAVLINK
# ============================================================================
def process_message(msg, data: DroneData, lat_filter: LowPassFilter, lon_filter: LowPassFilter, current_time: float):
    msg_type = msg.get_type()
    if msg_type == "GLOBAL_POSITION_INT":
        try:
            data.lat = lat_filter.update(msg.lat / 1e7, current_time)
            data.lon = lon_filter.update(msg.lon / 1e7, current_time)
            data.alt_agl = msg.relative_alt / 1000.0
            # Dipertahankan sama seperti drop.cpp.
            data.vx_east = msg.vx / 100.0
            data.vy_north = msg.vy / 100.0
            data.vz = -msg.vz / 100.0
            data.groundspeed = math.hypot(data.vx_east, data.vy_north)
            # Ground heading dari GPS velocity.
            if data.groundspeed > 0.1:
                heading = math.degrees(math.atan2(data.vx_east, data.vy_north))
                if heading < 0.0:
                    heading += 360.0

                data.heading = heading
            data.valid_lat = True
            data.valid_lon = True
            data.valid_velocity = all(math.isfinite(value) for value in (data.vx_east, data.vy_north, data.vz, data.groundspeed))
            return current_time
        except (AttributeError, TypeError, ValueError):
            return None
        
    if msg_type == "MISSION_CURRENT":
        try:
            data.wp = int(msg.seq)
        except (AttributeError, TypeError, ValueError):
            pass
    elif msg_type == "HEARTBEAT":
        try:
            if int(msg.custom_mode) == 10:
                data.mode = "AUTO"
            else:
                data.mode = "OTHER"
        except (AttributeError, TypeError, ValueError):
            data.mode = "OTHER"
    # VFR_HUD sengaja TIDAK digunakan.
    return None

def main():
    master = connect_pixhawk()
    if master is None:
        return 1

    telemetry_socket, telemetry_destination = (create_telemetry_socket())
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
    system_id = master.target_system
    component_id = master.target_component
    # INIT TUTUP SERVO
    send_servo_command(master, system_id, component_id, SERVO_DOOR_CHANNEL, PWM_TUTUP)
    send_servo_command(master, system_id, component_id, SERVO_CHANNEL, 1200)
    try:
        while True:
            current_time = time.monotonic()
            # ------------------------------------------------------------
            # MAVLINK POLLING
            # ------------------------------------------------------------
            messages_processed = 0
            while True:
                msg = master.recv_match(blocking=False)
                if msg is None:
                    break

                messages_processed += 1
                update_time = process_message(msg, data, lat_filter, lon_filter, current_time)
                if update_time is not None:
                    last_position_update_time = (update_time)
            if messages_processed == 0:
                time.sleep(0.005)

            current_time = time.monotonic()
            # ------------------------------------------------------------
            # PUBLISH TELEMETRY KE STR
            # ------------------------------------------------------------
            position_fresh = (data.valid_lat and data.valid_lon and last_position_update_time > 0.0 and (current_time - last_position_update_time <= POSITION_TIMEOUT))
            if (telemetry_socket is not None and position_fresh and (current_time - last_udp_publish_time >= TELEMETRY_PUBLISH_INTERVAL)):
                publish_telemetry(telemetry_socket, telemetry_destination, data)
                last_udp_publish_time = current_time

            if not position_fresh:
                print("Menunggu Sinyal GPS/posisi realtime...   ", end="\r", flush=True)
                continue

            # ------------------------------------------------------------
            # PREDICTION
            # ------------------------------------------------------------
            if (current_time - last_prediction_time >= PREDICTION_INTERVAL):
                last_prediction_time = current_time
                # Tanpa airspeed:
                # gunakan fixed/assumed wind.
                wind_speed = FIXED_WIND_SPEED
                wind_direction = FIXED_WIND_DIRECTION
                pred = predict_drop(data.lat, data.lon, data.alt_agl, data.vx_east, data.vy_north, data.vz, wind_speed, wind_direction)
                dynamic_drop_travel = (calculate_release_travel_distance(data, pred.fall_time))
                dynamic_impact_radius = (calculate_dynamic_impact_radius(data, pred.fall_time))
                dist_to_door = haversine(data.lat, data.lon, LAT_DOOR, LON_DOOR)
                raw_dist_to_drop = haversine(data.lat, data.lon, LAT_TARGET, LON_TARGET)
                pred_dist_to_drop = haversine(pred.lat_drop, pred.lon_drop, LAT_TARGET, LON_TARGET)
                # --------------------------------------------------------
                # PRINT
                # --------------------------------------------------------
                if (current_time - last_print_time >= PRINT_INTERVAL):
                    print()
                    print("========================================")
                    print(f"Posisi   : " f"{data.lat:.7f}, " f"{data.lon:.7f}")
                    print(f"WP       : {data.wp}" f"\t | Mode : {data.mode}")
                    print(
                        f"Alt Rel  : {data.alt_agl:.2f} m"
                        f"\t | Groundspeed : " f"{data.groundspeed:.2f} m/s")
                    print(f"Ground Hdg : " f"{data.heading:.2f} deg")
                    print(
                        f"Wind Assumed : " f"{wind_speed:.2f} m/s"
                        f"\t | Dir : " f"{wind_direction:.2f} deg")
                    print(f"Jarak ke Titik Pintu : " f"{dist_to_door:.2f} m")
                    print(f"Jarak GPS ke Target  : " f"{raw_dist_to_drop:.2f} m")
                    print(f"Jarak Prediksi Target: " f"{pred_dist_to_drop:.2f} m")
                    print(f"Fall Time             : " f"{pred.fall_time:.2f} s")
                    print(f"Jarak Tempuh Jatuh    : " f"{dynamic_drop_travel:.2f} m")
                    print(f"Radius Impact Dinamis : " f"{dynamic_impact_radius:.2f} m")
                    print(f"Drop Gate             : " f"{'ARMED' if drop_gate_armed else 'WAIT'}")
                    print(
                        f"Status   : Door = " f"{'OPEN' if door_opened else 'CLOSED'}"
                        f" | Drop = " f"{'DONE' if drop_done else 'WAIT'}")
                    print("========================================")
                    last_print_time = current_time
                # --------------------------------------------------------
                # A. BUKA PINTU CH6
                # --------------------------------------------------------
                waypointdoor_ok = (data.wp == DOOR_WP_INDEX)
                door_condition = (dist_to_door <= DOOR_OPEN_RADIUS and waypointdoor_ok and not door_opened)
                if door_condition:
                    print("\n=== KOORDINAT PINTU TERCAPAI! " "MEMBUKA PINTU (CH 6) ===")
                    send_servo_command(master, system_id, component_id, SERVO_DOOR_CHANNEL, PWM_BUKA)
                    door_opened = True
                # --------------------------------------------------------
                # B. WP DROP HANYA MISSION GATE
                # --------------------------------------------------------
                if (not drop_gate_armed and data.wp >= DROP_WP_INDEX):
                    drop_gate_armed = True
                    print("\n=== DROP MISSION GATE ARMED ===")
                    print(f"WP Gate         : {data.wp}")
                    print(f"Target Asli     : " f"{LAT_TARGET}, {LON_TARGET}")
                    print("CH5 Trigger     : " "PREDICTED IMPACT")
                # --------------------------------------------------------
                # C. KONDISI DROP
                # --------------------------------------------------------
                prediction_ok = (dynamic_impact_radius > 0.0 and math.isfinite(pred_dist_to_drop) and (pred_dist_to_drop <= dynamic_impact_radius))
                altitude_ok = (math.isfinite(data.alt_agl) and data.alt_agl >= MIN_DROP_ALTITUDE)
                mode_ok = (data.mode == "AUTO")
                telemetry_ok = (data.valid_velocity and math.isfinite(data.groundspeed) and math.isfinite(data.vx_east) and math.isfinite(data.vy_north) and math.isfinite(data.vz))
                target_condition = (drop_gate_armed and prediction_ok and telemetry_ok and mode_ok and altitude_ok and not drop_done)
                if target_condition:
                    print("\n=== PREDICTED IMPACT " "MEMENUHI TARGET ===")
                    print(f"WP Gate         : " f"{data.wp} (sudah ARM)")
                    print(f"Jarak GPS Target: " f"{raw_dist_to_drop:.2f} m")
                    print(f"Predicted Impact: " f"{pred_dist_to_drop:.2f} m")
                    print(f"Radius Impact   : " f"{dynamic_impact_radius:.2f} m")
                    print(f"Jarak Tempuh    : " f"{dynamic_drop_travel:.2f} m")
                    print(f"Fall Time       : " f"{pred.fall_time:.2f} s")
                    print(f"Altitude AGL    : " f"{data.alt_agl:.2f} m")
                    print(f"Groundspeed     : " f"{data.groundspeed:.2f} m/s")
                    print(f"Assumed Wind    : " f"{wind_speed:.2f} m/s @ " f"{wind_direction:.2f} deg")
                    print(f"Mode            : " f"{data.mode}")
                    # ----------------------------------------------------
                    # 1. RELEASE CH5
                    # ----------------------------------------------------
                    print("=== MELEPAS PAYLOAD (CH 5) ===")
                    send_servo_command(master, system_id, component_id, SERVO_CHANNEL, PWM_BUKA)
                    drop_done = True
                    # ----------------------------------------------------
                    # 2. TIMER PENUTUPAN CH6
                    # ----------------------------------------------------
                    door_close_pending = True
                    door_close_time = (current_time + DOOR_CLOSE_DELAY)
                    print("=== PINTU CH6 AKAN DITUTUP " f"DALAM {DOOR_CLOSE_DELAY:.0f} DETIK ===")

                # --------------------------------------------------------
                # AUTO CLOSE PINTU SETELAH DROP
                # --------------------------------------------------------
                if (door_close_pending and current_time >= door_close_time):
                    send_servo_command(master, system_id, component_id, SERVO_DOOR_CHANNEL, PWM_TUTUP)
                    door_opened = False
                    door_close_pending = False
                    print("\n=== DELAY SELESAI ===")
                    print("=== PINTU PAYLOAD CH6 " "DITUTUP KEMBALI ===")
                    if (drop_done and not door_opened and not door_close_pending):
                        print("\n========================================")
                        print("=== SEQUENCE PAYLOAD SELESAI ===")
                        print("=== CH5 : PAYLOAD SUDAH DROP       ===")
                        print("=== CH6 : PINTU SUDAH DITUTUP       ===")
                        print("=== PROGRAM BERHENTI               ===")
                        print("========================================")
                        return 0
                    
    except KeyboardInterrupt:
        print("\n\nProgram dihentikan oleh user.")
        return 0
    except Exception as exc:
        print(f"\nERROR runtime: {exc}", file=sys.stderr,)
        return 1
    finally:
        if telemetry_socket is not None:
            try:
                telemetry_socket.close()
            except Exception:
                pass

        try:
            master.close()
        except Exception:
            pass

if __name__ == "__main__":
    sys.exit(main())
