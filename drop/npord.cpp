#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstring>
#include <string>
#include <chrono>
#include <algorithm>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <mavlink/common/mavlink.h>

// PARAMETER KONEKSI & SERVO
#define SERIAL_PORT "/dev/ttyS7"
const int BAUDRATE = B115200;

// TELEMETRY IPC KE STR VIA UDP LOOPBACK
// Hanya npord yang membuka /dev/ttyS7. str menerima telemetry dari sini.
const char* TELEMETRY_IP = "127.0.0.1";
const int TELEMETRY_PORT = 5000;
const double TELEMETRY_PUBLISH_INTERVAL = 0.10; // 10 Hz
const double POSITION_TIMEOUT = 1.0;             // posisi GLOBAL_POSITION_INT >1s dianggap basi
const int SERVO_CHANNEL = 5;       // Servo Drop Payload
const int SERVO_DOOR_CHANNEL = 6;  // Servo Pintu Payload
const float PWM_BUKA  = 2000.0f;
const float PWM_TUTUP = 1000.0f;
const float SERVO_FEEDBACK_TOLERANCE = 40.0f;
const double SERVO_FEEDBACK_TIMEOUT = 1.0;

// KOORDINAT TARGET PINTU & DROP
const double LAT_DOOR   = -0.9310986;  // Koordinat khusus untuk membuka pintu
const double LON_DOOR   = 116.8483114;
// const double LAT_TARGET = -0.9316457;  // Koordinat khusus untuk menjatuhkan payload
// const double LON_TARGET = 116.8492448;
const double LAT_TARGET = -0.9319386;  // Koordinat khusus untuk menjatuhkan payload
const double LON_TARGET = 116.8497276;

// WP aktual dibaca dari MAVLINK_MSG_ID_MISSION_CURRENT.
// DROP_WP_INDEX hanya mission gate; koordinat target payload tetap LAT_TARGET/LON_TARGET.
// Untuk ArduPlane, custom_mode 10 = AUTO.
const int DOOR_WP_INDEX = 16;
const int DROP_WP_INDEX = 17;
const double DOOR_OPEN_RADIUS = 26.0;   // meter (Radius toleransi koordinat pintu)
// Radius drop tidak lagi statis.
// Ambang drop dihitung real-time dari telemetry:
// groundspeed + airspeed + waktu jatuh hasil simulasi payload.
const double MIN_DROP_ALTITUDE = 100.0; // meter

// FISIKA & SIMULASI
const double G = 9.81;
const double AIR_DENSITY = 1.225;
const double PAYLOAD_MASS = 0.5;
const double PAYLOAD_AREA = 0.01;
const double DRAG_COEFFICIENT = 1.0;
const double SIMULATION_DT = 0.01;
const double MAX_SIMULATION_TIME = 60.0;
const double SERVO_DELAY = 0.0;
// Delay setelah payload dilepas sebelum pintu payload ditutup
const double DOOR_CLOSE_DELAY = 15; // detik

// TIMING & TIMEOUT
const double PRINT_INTERVAL = 0.2;
const double PREDICTION_INTERVAL = 0.10;

// LPF TAU
const double LPF_TAU_POSITION = 0.20;
const double LPF_TAU_VELOCITY = 0.15;
const double LPF_TAU_AIRSPEED = 0.15;
const double LPF_TAU_ALTITUDE = 0.20;

// HELPER WAKTU
double get_time_monotonic() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now.time_since_epoch()).count();
}

// LOW PASS FILTER
class LowPassFilter {
    private:
        double tau;
        double value;
        double timestamp;
        bool initialized;

    public:
        LowPassFilter(double tau) : tau(tau), initialized(false), value(0.0), timestamp(0.0) {}
        double update(double new_value, double current_time) {
            if (!initialized) {
                value = new_value;
                timestamp = current_time;
                initialized = true;
                return value;
            }

            double dt = current_time - timestamp;
            timestamp = current_time;
            dt = std::max(0.0, std::min(dt, 1.0));
            double alpha = (dt > 0.0) ? dt / (tau + dt) : 0.0;
            value += alpha * (new_value - value);
            return value;
        }
        double get_value() const { return value; }
};

// WIND ESTIMATOR
// Vwind = Vground - Vair.
// Asumsi: heading 0°=North, 90°=East dan airspeed sejajar heading.
// direction di sini adalah arah gerak angin (toward).
struct WindEstimate {
    double east = 0.0;
    double north = 0.0;
    double speed = 0.0;
    double direction = 0.0;
    bool valid = false;
};

WindEstimate estimate_wind_from_airspeed(double vx_east, double vy_north, double airspeed, double heading_deg) {
    WindEstimate result;
    if (!std::isfinite(vx_east) || !std::isfinite(vy_north) || !std::isfinite(airspeed) || !std::isfinite(heading_deg) || airspeed < 0.0 || heading_deg < 0.0 || heading_deg >= 360.0) {
        return result;
    }

    const double heading_rad = heading_deg * M_PI / 180.0;
    const double v_air_east = airspeed * std::sin(heading_rad);
    const double v_air_north = airspeed * std::cos(heading_rad);
    result.east = vx_east - v_air_east;
    result.north = vy_north - v_air_north;
    result.speed = std::hypot(result.east, result.north);
    if (result.speed > 1e-6) {
        result.direction = std::atan2(result.east, result.north) * 180.0 / M_PI;
        if (result.direction < 0.0)
            result.direction += 360.0;
    }

    result.valid = true;
    return result;
}

// STRUCT DATA
struct DroneData {
    double lat = 0, lon = 0, alt_msl = 0, alt_agl = 0;
    double groundspeed = 0, heading = 0;
    double vx_east = 0, vy_north = 0, vz = 0;
    double airspeed = 0, wind_speed = 0, wind_direction = 0;
    int wp = -1;
    std::string mode = "UNKNOWN";
    bool valid_lat = false, valid_lon = false;
    bool valid_airspeed = false;
    bool valid_heading = false;
    bool valid_wind = false;
};

struct ImpactResult {
    double x, y, z, time, vx, vy, vz;
};

struct PredictionResult {
    double lat_drop, lon_drop, fall_time, dx, dy;
    double release_lat, release_lon;
    double impact_vx, impact_vy, impact_vz;
};

// HAVERSINE
double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;
    double phi1 = lat1 * M_PI / 180.0;
    double phi2 = lat2 * M_PI / 180.0;
    double dphi = (lat2 - lat1) * M_PI / 180.0;
    double dlambda = (lon2 - lon1) * M_PI / 180.0;
    double a = std::sin(dphi / 2.0) * std::sin(dphi / 2.0) + std::cos(phi1) * std::cos(phi2) * std::sin(dlambda / 2.0) * std::sin(dlambda / 2.0);
    a = std::max(0.0, std::min(1.0, a));
    return 2.0 * R * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

// KONVERSI METER KE GPS
void meter_to_latlon(double lat, double lon, double dx, double dy, double &out_lat, double &out_lon) {
    double dlat = dy / 111320.0;
    double cos_lat = std::cos(lat * M_PI / 180.0);
    if (std::abs(cos_lat) < 1e-6) cos_lat = 1e-6;
    double dlon = dx / (111320.0 * cos_lat);
    out_lat = lat + dlat;
    out_lon = lon + dlon;
}

// SIMULASI DRAG + WIND
ImpactResult simulate_payload_impact(double alt, double vx_init, double vy_init, double vz_init, double wind_speed, double wind_dir) {
    double x = 0.0, y = 0.0, z = alt;
    double vx = vx_init, vy = vy_init, vz = vz_init;
    double theta = wind_dir * M_PI / 180.0;
    double wx = wind_speed * std::sin(theta);
    double wy = wind_speed * std::cos(theta);
    double t = 0.0;
    while (z > 0.0 && t < MAX_SIMULATION_TIME) {
        double vrel_x = vx - wx;
        double vrel_y = vy - wy;
        double vrel_z = vz;
        double vrel = std::sqrt(vrel_x * vrel_x + vrel_y * vrel_y + vrel_z * vrel_z);
        double force_drag_x = 0, force_drag_y = 0, force_drag_z = 0;
        if (vrel > 0.0001) {
            double drag_factor = -0.5 * AIR_DENSITY * DRAG_COEFFICIENT * PAYLOAD_AREA * vrel;
            force_drag_x = drag_factor * vrel_x;
            force_drag_y = drag_factor * vrel_y;
            force_drag_z = drag_factor * vrel_z;
        }

        double acc_x = force_drag_x / PAYLOAD_MASS;
        double acc_y = force_drag_y / PAYLOAD_MASS;
        double acc_z = -G + (force_drag_z / PAYLOAD_MASS);
        vx += acc_x * SIMULATION_DT;
        vy += acc_y * SIMULATION_DT;
        vz += acc_z * SIMULATION_DT;
        x += vx * SIMULATION_DT;
        y += vy * SIMULATION_DT;
        z += vz * SIMULATION_DT;
        t += SIMULATION_DT;
    }
    if (z < 0.0) z = 0.0;
    return {x, y, z, t, vx, vy, vz};
}

// PREDIKSI TITIK JATUH
PredictionResult predict_drop(double lat, double lon, double alt, double vx_east, double vy_north, double vz, double wind_speed, double wind_dir) {
    double dx_release = vx_east * SERVO_DELAY;
    double dy_release = vy_north * SERVO_DELAY;
    double alt_release = alt + vz * SERVO_DELAY;
    if (alt_release <= 0.0) alt_release = 0.1;
    double rel_lat, rel_lon;
    meter_to_latlon(lat, lon, dx_release, dy_release, rel_lat, rel_lon);
    ImpactResult impact = simulate_payload_impact(alt_release, vx_east, vy_north, vz, wind_speed, wind_dir);
    double drop_lat, drop_lon;
    meter_to_latlon(rel_lat, rel_lon, impact.x, impact.y, drop_lat, drop_lon);
    return {drop_lat, drop_lon, impact.time, dx_release + impact.x, dy_release + impact.y, rel_lat, rel_lon, impact.vx, impact.vy, impact.vz};
}

// ============================================================================
// DYNAMIC IMPACT TOLERANCE + RELEASE TRAVEL
// ----------------------------------------------------------------------------
// WP TIDAK MENJADI RADIUS DROP. WP18 hanya dipakai untuk meng-arm sistem.
// Titik target tetap LAT_TARGET/LON_TARGET.
//
// 1) Jarak tempuh nominal selama payload jatuh:
//      D_travel = GS * t_f
//
// 2) Propagasi ketidakpastian jarak:
//      sigma_GSdist = t_f * sigma_GS
//      sigma_tdist  = GS * sigma_t
//      sigma_travel = sqrt(sigma_GSdist^2 + sigma_tdist^2)
//
// 3) Tambahkan ketidakpastian model trajectory:
//      sigma_total = sqrt(sigma_model^2 + sigma_travel^2)
//
// 4) Radius keputusan predicted impact:
//      R_dynamic = clamp(R_base + K_sigma*sigma_total, R_min, R_max)
//
// R_dynamic adalah toleransi prediksi impact, BUKAN radius acceptance WP.
// ============================================================================
const double DYNAMIC_RADIUS_BASE = 3.0;       // m
const double DYNAMIC_RADIUS_MIN  = 5.0;       // m
const double DYNAMIC_RADIUS_MAX  = 25.0;      // m, cap toleransi impact (bukan acceptance radius WP)
const double DYNAMIC_RADIUS_KSIGMA = 2.0;     // 2-sigma
const double SIGMA_GROUNDSPEED = 1.0;         // m/s
const double SIGMA_FALL_TIME = 0.30;          // s
const double SIGMA_MODEL_PREDICTION = 5.0;    // m

double calculate_release_travel_distance(const DroneData &data, double fall_time) {
    if (!std::isfinite(data.groundspeed) || !std::isfinite(fall_time) || data.groundspeed < 0.0 || fall_time <= 0.0) {
        return 0.0;
    }
    return data.groundspeed * fall_time;
}

double calculate_dynamic_impact_radius(const DroneData &data, double fall_time) {
    if (!std::isfinite(data.groundspeed) || !std::isfinite(fall_time) || data.groundspeed < 0.0 || fall_time <= 0.0) {
        return DYNAMIC_RADIUS_MIN;
    }

    const double sigma_from_gs = fall_time * SIGMA_GROUNDSPEED;
    const double sigma_from_time = data.groundspeed * SIGMA_FALL_TIME;
    const double sigma_travel = std::sqrt(sigma_from_gs * sigma_from_gs + sigma_from_time * sigma_from_time);
    const double sigma_total = std::sqrt(SIGMA_MODEL_PREDICTION * SIGMA_MODEL_PREDICTION + sigma_travel * sigma_travel);
    const double radius = DYNAMIC_RADIUS_BASE + DYNAMIC_RADIUS_KSIGMA * sigma_total;
    return std::max(DYNAMIC_RADIUS_MIN, std::min(DYNAMIC_RADIUS_MAX, radius));
}

// ============================================================================
// UDP TELEMETRY PUBLISHER -> STR
// ============================================================================
int create_telemetry_udp_socket(struct sockaddr_in &dest) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cerr << "UDP telemetry socket gagal: " << strerror(errno) << std::endl;
        return -1;
    }

    std::memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(TELEMETRY_PORT);
    if (inet_pton(AF_INET, TELEMETRY_IP, &dest.sin_addr) != 1) {
        std::cerr << "Alamat telemetry UDP tidak valid: " << TELEMETRY_IP << std::endl;
        close(fd);
        return -1;
    }

    std::cout << "[UDP] Telemetry -> " << TELEMETRY_IP << ":" << TELEMETRY_PORT << std::endl;
    return fd;
}

void publish_telemetry(int udp_fd, const struct sockaddr_in &dest, const DroneData &data) {
    if (udp_fd < 0 || !data.valid_lat || !data.valid_lon)
        return;

    char packet[256];
    // Desain A: kirim HANYA posisi realtime UAV ke str.
    // WP tetap digunakan internal oleh npord untuk logika payload,
    // tetapi tidak menjadi input gate deteksi kamera.
    int len = std::snprintf(packet, sizeof(packet), "LAT=%.8f LON=%.8f", data.lat, data.lon);
    if (len <= 0 || static_cast<size_t>(len) >= sizeof(packet))
        return;

    ssize_t sent = sendto(udp_fd, packet, static_cast<size_t>(len), 0, reinterpret_cast<const sockaddr*>(&dest), sizeof(dest));
    if (sent < 0) {
        // Receiver boleh belum hidup; npord tetap lanjut mengerjakan payload.
        static bool warned = false;
        if (!warned) {
            std::cerr << "[UDP] Gagal mengirim telemetry ke str: " << strerror(errno) << std::endl;
            warned = true;
        }
    }
}

// SERIAL COMMANDS
int open_serial(const char *port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "open(" << port << ") gagal: " << strerror(errno) << std::endl;
        return -1;
    }

    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "tcgetattr gagal: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, BAUDRATE);
    cfsetospeed(&tty, BAUDRATE);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr gagal: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

void send_servo_command(int serial_fd, uint8_t sys_id, uint8_t comp_id, int channel, float pwm) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(255, 190, &msg, sys_id, comp_id, MAV_CMD_DO_SET_SERVO, 0, channel, pwm, 0,0,0,0,0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    write(serial_fd, buf, len);
}

int main() {
    std::cout << "Connecting to Pixhawk..." << std::endl;
    int serial_fd = open_serial(SERIAL_PORT);
    if (serial_fd < 0) {
        std::cerr << "Gagal membuka port serial!" << std::endl;
        return 1;
    }

    uint8_t system_id = 1, component_id = 1;
    // WAIT HEARTBEAT
    mavlink_message_t msg{};
    mavlink_status_t status{};
    bool connected = false;
    double start_time = get_time_monotonic();
    while (!connected) {
        uint8_t buf[512];
        int n = read(serial_fd, buf, sizeof(buf));
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        std::cout << " <-- HEARTBEAT";
                    }
                    std::cout << std::endl;
                    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        system_id = msg.sysid;
                        component_id = msg.compid;
                        connected = true;
                        break;
                    }
                }
            }
        }

        if (get_time_monotonic() - start_time > 10.0) {
            std::cerr << "\nERROR: Tidak menerima HEARTBEAT dari Pixhawk selama 10 detik." << std::endl;
            close(serial_fd);
            return 1;
        }

        usleep(10000);
    }
    std::cout << "Connected to Pixhawk!" << std::endl;
    struct sockaddr_in telemetry_dest{};
    int telemetry_udp_fd = create_telemetry_udp_socket(telemetry_dest);
    if (telemetry_udp_fd < 0) {
        std::cerr << "Peringatan: telemetry UDP tidak aktif; npord tetap berjalan." << std::endl;
    }

    LowPassFilter lat_filter(LPF_TAU_POSITION), lon_filter(LPF_TAU_POSITION);
    DroneData data;
    bool drop_done = false;
    bool door_opened = false;
    bool drop_gate_armed = false;
    // Tidak ada force-drop. Setelah gate aktif, CH5 hanya dipicu oleh predicted impact.
    // State untuk penutupan otomatis pintu setelah drop
    bool door_close_pending = false;
    double door_close_time = 0.0;
    double last_print_time = 0.0;
    double last_prediction_time = 0.0;
    double last_udp_publish_time = 0.0;
    double last_position_update_time = 0.0;
    // INIT TUTUP SERVO
    send_servo_command(serial_fd, system_id, component_id, SERVO_DOOR_CHANNEL, 1000);
    send_servo_command(serial_fd, system_id, component_id, SERVO_CHANNEL, 1200);
    while (true) {
        uint8_t buffer[512];
        int bytes_read = read(serial_fd, buffer, sizeof(buffer));
        double current_time = get_time_monotonic();
        if (bytes_read > 0) {
            for (int i = 0; i < bytes_read; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                    if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                        mavlink_global_position_int_t pos;
                        mavlink_msg_global_position_int_decode(&msg, &pos);
                        data.lat = lat_filter.update(pos.lat / 1e7, current_time);
                        data.lon = lon_filter.update(pos.lon / 1e7, current_time);
                        data.alt_agl = pos.relative_alt / 1000.0; 
                        data.vx_east = pos.vx / 100.0;
                        data.vy_north = pos.vy / 100.0;
                        data.vz = -pos.vz / 100.0;
                        data.groundspeed = std::hypot(data.vx_east, data.vy_north);
                        data.valid_lat = data.valid_lon = true;
                        last_position_update_time = current_time;
                    } else if (msg.msgid == MAVLINK_MSG_ID_MISSION_CURRENT) {
                        mavlink_mission_current_t m;
                        mavlink_msg_mission_current_decode(&msg, &m);
                        data.wp = m.seq;
                    } else if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        mavlink_heartbeat_t hb;
                        mavlink_msg_heartbeat_decode(&msg, &hb);
                        if (hb.custom_mode == 10) data.mode = "AUTO"; 
                        else data.mode = "OTHER";
                    } else if (msg.msgid == MAVLINK_MSG_ID_VFR_HUD) {
                        mavlink_vfr_hud_t hud;
                        mavlink_msg_vfr_hud_decode(&msg, &hud);
                        data.airspeed = hud.airspeed;
                        data.groundspeed = hud.groundspeed;
                        data.heading = hud.heading;
                        data.valid_airspeed = std::isfinite(data.airspeed) && data.airspeed >= 0.0;
                        data.valid_heading = std::isfinite(data.heading) && data.heading >= 0.0 && data.heading < 360.0;
                    }
                }
            }
        } else {
            // Tidak ada byte baru kali ini. Tetap beri kesempatan publisher UDP
            // berjalan dengan data GPS terakhir yang valid.
            usleep(5000);
        }
        // Publish telemetry ke str maksimum 10 Hz. /dev/ttyS7 tetap hanya
        // dibaca oleh npord; str tidak lagi membuka port serial ini.
        // Jangan kirim posisi lama terlalu lama: jika GLOBAL_POSITION_INT
        // tidak diperbarui > POSITION_TIMEOUT, publisher berhenti mengirim.
        const bool position_fresh = data.valid_lat && data.valid_lon && last_position_update_time > 0.0 && (current_time - last_position_update_time <= POSITION_TIMEOUT);
        if (telemetry_udp_fd >= 0 && position_fresh &&
            current_time - last_udp_publish_time >= TELEMETRY_PUBLISH_INTERVAL) {
            publish_telemetry(telemetry_udp_fd, telemetry_dest, data);
            last_udp_publish_time = current_time;
        }
        
        // Untuk logika payload, jika tidak ada GPS valid/jika data posisi basi jangan lanjut.
        if (!position_fresh) {
            std::cout << "Menunggu Sinyal GPS/posisi realtime...   " << std::flush;
            continue;
        }

        if (current_time - last_prediction_time >= PREDICTION_INTERVAL) {
            last_prediction_time = current_time;
            // 1. Hitung Prediksi (Fisika)
            // Estimasi wind dari airspeed + GPS ground velocity + heading.
            WindEstimate wind = estimate_wind_from_airspeed(data.vx_east, data.vy_north, data.airspeed, data.heading);
            data.valid_wind = data.valid_airspeed && data.valid_heading && wind.valid;
            const double wind_speed = data.valid_wind ? wind.speed : 0.0;
            const double wind_direction = data.valid_wind ? wind.direction : 0.0;
            PredictionResult pred = predict_drop(data.lat, data.lon, data.alt_agl, data.vx_east, data.vy_north, data.vz, wind_speed, wind_direction);
            // Dua nilai dipisahkan:
            // 1. release_travel = jarak tempuh nominal selama payload jatuh
            // 2. impact_radius = toleransi ketidakpastian predicted impact
            const double dynamic_drop_travel = calculate_release_travel_distance(data, pred.fall_time);
            const double dynamic_impact_radius = calculate_dynamic_impact_radius(data, pred.fall_time);
            // 2. Hitung Jarak
            double dist_to_door = haversine(data.lat, data.lon, LAT_DOOR, LON_DOOR); // Jarak GPS ke titik Pintu
            double raw_dist_to_drop = haversine(data.lat, data.lon, LAT_TARGET, LON_TARGET); // Jarak GPS murni ke target Drop
            double pred_dist_to_drop = haversine(pred.lat_drop, pred.lon_drop, LAT_TARGET, LON_TARGET); // Jarak prediksi peluru ke target Drop
            if (current_time - last_print_time >= PRINT_INTERVAL) {
                std::cout << std::fixed << std::setprecision(2);
                std::cout << "\n========================================" << std::endl;
                std::cout << "Posisi   : " << std::setprecision(7) << data.lat << ", " << data.lon
                          << std::setprecision(2) << std::endl;
                std::cout << "WP       : " << data.wp
                          << " \t | Mode : " << data.mode << std::endl;
                std::cout << "Alt Rel  : " << data.alt_agl << " m"
                          << " \t | Airspeed : " << data.airspeed << " m/s"
                          << " \t | Heading : " << data.heading << " deg" << std::endl;
                if (data.valid_wind) {
                    std::cout << "Wind Est : " << wind.speed << " m/s \t | Der : " << wind.direction << " deg " << std::endl;
                } else {
                    std::cout << "Wind Est : INVALID (airspeed/heading belum valid)" << std::endl;
                }
                std::cout << "Jarak ke Titik Pintu : " << dist_to_door << " m" << std::endl;
                std::cout << "Jarak GPS ke Target  : " << raw_dist_to_drop << " m" << std::endl;
                std::cout << "Jarak Prediksi Target: " << pred_dist_to_drop << " m" << std::endl;
                std::cout << "Fall Time             : " << pred.fall_time << " s" << std::endl;
                std::cout << "Jarak Tempuh Jatuh    : " << dynamic_drop_travel << " m" << std::endl;
                std::cout << "Radius Impact Dinamis : " << dynamic_impact_radius << " m" << std::endl;
                std::cout << "Drop Gate             : " << (drop_gate_armed ? "ARMED" : "WAIT") << std::endl;
                std::cout << "Status   : Door = " << (door_opened ? "OPEN" : "CLOSED") << " | Drop = " << (drop_done ? "DONE" : "WAIT") << std::endl;
                std::cout << "========================================" << std::endl;
                last_print_time = current_time;
            }
            // A. KONDISI BUKA PINTU
            bool waypointdoor_ok = (data.wp == DOOR_WP_INDEX);
            bool door_condition = (dist_to_door <= DOOR_OPEN_RADIUS) && waypointdoor_ok && !door_opened;
            if (door_condition) {
                std::cout << "\n=== KOORDINAT PINTU TERCAPAI! MEMBUKA PINTU (CH 6) ===" << std::endl;
                send_servo_command(serial_fd, system_id, component_id, SERVO_DOOR_CHANNEL, 2000);
                door_opened = true;
            }
            // B. WP DROP HANYA SEBAGAI MISSION GATE
            // ------------------------------------------------------------
            // Setelah WP18 tercapai, gate tetap ARM walaupun wahana
            // kemudian bergerak ke WP berikutnya. WP tidak lagi menjadi
            // syarat posisi CH5.
            if (!drop_gate_armed && data.wp >= DROP_WP_INDEX) {
                drop_gate_armed = true;
                std::cout << "\n=== DROP MISSION GATE ARMED ===" << std::endl;
                std::cout << "WP Gate         : " << data.wp << std::endl;
                std::cout << "Target Asli     : " << LAT_TARGET << ", " << LON_TARGET << std::endl;
                std::cout << "CH5 Trigger     : PREDICTED IMPACT" << std::endl;
            }
            // C. KONDISI DROP PAYLOAD
            // Satu-satunya syarat geometris adalah predicted impact
            // terhadap target asli.
            bool prediction_ok = (dynamic_impact_radius > 0.0) && std::isfinite(pred_dist_to_drop) && (pred_dist_to_drop <= dynamic_impact_radius);
            bool altitude_ok = std::isfinite(data.alt_agl) && (data.alt_agl >= MIN_DROP_ALTITUDE);
            bool mode_ok = (data.mode == "AUTO");
            // Prediksi tidak boleh dianggap valid bila input penerbangan utama
            // belum valid. Tidak ada force-drop fallback.
            bool telemetry_ok = data.valid_airspeed && data.valid_heading && std::isfinite(data.groundspeed) && std::isfinite(data.vx_east) && std::isfinite(data.vy_north) && std::isfinite(data.vz);
            bool target_condition = drop_gate_armed && prediction_ok && telemetry_ok && mode_ok && altitude_ok && !drop_done;
            if (target_condition) {
                std::cout << "\n=== PREDICTED IMPACT MEMENUHI TARGET ===" << std::endl;
                std::cout << "WP Gate         : " << data.wp << " (sudah ARM)" << std::endl;
                std::cout << "Jarak GPS Target: " << raw_dist_to_drop << " m" << std::endl;
                std::cout << "Predicted Impact: " << pred_dist_to_drop << " m" << std::endl;
                std::cout << "Radius Impact   : " << dynamic_impact_radius << " m" << std::endl;
                std::cout << "Jarak Tempuh    : " << dynamic_drop_travel << " m" << std::endl;
                std::cout << "Fall Time       : " << pred.fall_time << " s" << std::endl;
                std::cout << "Altitude AGL    : " << data.alt_agl << " m" << std::endl;
                std::cout << "Airspeed        : " << data.airspeed << " m/s" << std::endl;
                std::cout << "Groundspeed     : " << data.groundspeed << " m/s" << std::endl;
                std::cout << "Mode            : " << data.mode << std::endl;
                // ========================================================
                // 1. RELEASE CH5
                // ========================================================
                std::cout << "=== MELEPAS PAYLOAD (CH 5) ===" << std::endl;
                send_servo_command(serial_fd, system_id, component_id, SERVO_CHANNEL, 2000);
                drop_done = true;
                // ========================================================
                // 2. TIMER PENUTUPAN CH6
                // ========================================================
                door_close_pending = true;
                door_close_time = current_time + DOOR_CLOSE_DELAY;
                std::cout << "=== PINTU CH6 AKAN DITUTUP DALAM " << DOOR_CLOSE_DELAY << " DETIK ===" << std::endl;
            }

            // ================================================================
            // AUTO CLOSE PINTU PAYLOAD SETELAH DROP
            // ================================================================
            if (door_close_pending && current_time >= door_close_time) {
                send_servo_command(serial_fd, system_id, component_id, SERVO_DOOR_CHANNEL, 1000);
                door_opened = false;
                door_close_pending = false;
                std::cout << "\n=== DELAY SELESAI ===" << std::endl;
                std::cout << "=== PINTU PAYLOAD CH6 DITUTUP KEMBALI ===" << std::endl;
                // ============================================================
                // DROP SELESAI + PINTU SUDAH DITUTUP
                // PROGRAM BERHENTI
                // ============================================================
                if (drop_done && !door_opened && !door_close_pending) {
                    std::cout << "\n========================================" << std::endl;
                    std::cout << "=== SEQUENCE PAYLOAD SELESAI ===" << std::endl;
                    std::cout << "=== CH5 : PAYLOAD SUDAH DROP       ===" << std::endl;
                    std::cout << "=== CH6 : PINTU SUDAH DITUTUP     ===" << std::endl;
                    std::cout << "=== PROGRAM BERHENTI               ===" << std::endl;
                    std::cout << "========================================" << std::endl;
                    if (telemetry_udp_fd >= 0) close(telemetry_udp_fd);
                    close(serial_fd);
                    return 0;
                } 
            }
        } 
    } 
    if (telemetry_udp_fd >= 0) close(telemetry_udp_fd);
    close(serial_fd);
    return 0;
}