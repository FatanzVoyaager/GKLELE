// Streaming & Record with Detection non-ROS
#include <gst/gst.h>
#include <opencv2/opencv.hpp>
#include <gst/app/gstappsrc.h>
#include <thread>
#include <atomic>
#include <deque>
#include <csignal>
#include <ctime>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <chrono>
#include <algorithm>
#include <cerrno>
#include <glib-unix.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define WIDTH 640   
#define HEIGHT 480
#define FPS 30

// =============================================================
// GATE DETEKSI BERDASARKAN KOORDINAT (DESAIN A)
// =============================================================
// npord hanya mengirim posisi UAV realtime (LAT/LON) melalui UDP.
// Koordinat pusat area deteksi dan radius ditetapkan DI SINI.
// WP TIDAK DIGUNAKAN SEBAGAI SYARAT DETEKSI.
const double DETECTION_TARGETS[][2] = {
    {-0.9610288, 116.7048257},  // Target 1
    {-0.8992702, 116.7678433},  // Target 2
    {-0.9103463, 116.8101063},  // Target 3
    {-0.9319386, 116.8497276}   // Target 4
};
const int NUM_DETECTION_TARGETS = sizeof(DETECTION_TARGETS) / sizeof(DETECTION_TARGETS[0]);
const double DETECTION_RADIUS_M = 200.0;

// npord mengirim telemetry ke loopback UDP.
const char* TELEMETRY_BIND_IP = "127.0.0.1";
const int TELEMETRY_PORT = 5000;
const double TELEMETRY_TIMEOUT = 10.0; // >2 s tanpa data => detection OFF

GstElement *global_appsrc = NULL;
GMainLoop *loop = NULL;
char pipeline[2048];
std::atomic<bool> run(true);
std::atomic<int> numframe(0);
bool has_target = false;
std::atomic<double> current_lat(NAN);
std::atomic<double> current_lon(NAN);
std::atomic<bool> telemetry_valid(false);
std::atomic<bool> detection_enable(false);
std::atomic<double> last_telemetry_time(0.0);
std::atomic<double> current_detection_distance(NAN);
bool pagi = true;
bool siang = false;
bool sore = false;

#define SDELAY 2           // detik setelah hilang
#define BUFF_SEC 1         // buffer sebelum detect
#define ROTATE_SEC 3600    // rotate file

class FPSCounter {
private:
    int count = 0;
    double start = (double)cv::getTickCount();

    public:
    void update() {
        count++;
        double now = (double)cv::getTickCount();
        double sec = (now - start) / cv::getTickFrequency();
        if (sec >= 1.0) {
            printf("[FPS] %.2f\n", count / sec);
            count = 0;
            start = now;
        }
    }
};
FPSCounter fps;

gboolean on_sigint(gpointer) {
    printf("\n[INFO] Shutdown...\n");
    run = false;
    if (loop) g_main_loop_quit(loop);
    return G_SOURCE_REMOVE;
}

cv::VideoWriter create_writer(std::string name, time_t &start_time) {
    char filename[256];
    time_t now = time(0);
    strftime(filename, sizeof(filename), ("record/" + name + "_%Y%m%d_%H%M%S.mkv").c_str(), localtime(&now));
    int fourcc = cv::VideoWriter::fourcc('M','J','P','G');
    cv::VideoWriter writer(filename, fourcc, FPS, cv::Size(WIDTH, HEIGHT));
    if (!writer.isOpened())
        printf("[ERROR] Writer gagal: %s\n", filename);
    else
        printf("[INFO] Recording: %s\n", filename);
        
    start_time = time(0);
    return writer;
}

bool device_exists(int idx) {
    std::string dev = "/dev/video" + std::to_string(idx);
    struct stat st;
    return (stat(dev.c_str(), &st) == 0) && S_ISCHR(st.st_mode);
}

cv::VideoCapture open_camera(int &used_index) {
    for (int i = 0; i < 10; i++) {
        if (!device_exists(i))
            continue;

        cv::VideoCapture cap(i, cv::CAP_V4L2);
        if (cap.isOpened()) {
            printf("[INFO] Kamera ditemukan di /dev/video%d\n", i);
            cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
            cap.set(cv::CAP_PROP_FRAME_WIDTH, WIDTH);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, HEIGHT);
            cap.set(cv::CAP_PROP_FPS, FPS);
            used_index = i;
            return cap;
        }
    }
    used_index = -1;
    return cv::VideoCapture();
}

double angle(cv::Point p1, cv::Point p2, cv::Point p0) {
    double dx1 = p1.x - p0.x;
    double dy1 = p1.y - p0.y;
    double dx2 = p2.x - p0.x;
    double dy2 = p2.y - p0.y;
    double dot = dx1 * dx2 + dy1 * dy2;
    double mag1 = sqrt(dx1 * dx1 + dy1 * dy1);
    double mag2 = sqrt(dx2 * dx2 + dy2 * dy2);
    return dot / (mag1 * mag2 + 1e-10);
}

double getPercentile(const cv::Mat &channel, double percent) {
    CV_Assert(channel.type() == CV_8UC1);
    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange = {range};
    cv::Mat hist;
    cv::calcHist(&channel, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);
    int total = channel.rows * channel.cols;
    int threshold = (int)(percent * total);
    int cumulative = 0;
    for (int i = 0; i < 256; i++) {
        cumulative += (int)hist.at<float>(i);
        if (cumulative >= threshold)
        return i;
    }
    return 255;
}

double haversine_distance_m(double lat1, double lon1, double lat2, double lon2) {
    constexpr double R = 6371000.0;
    const double p1 = lat1 * M_PI / 180.0;
    const double p2 = lat2 * M_PI / 180.0;
    const double dp = (lat2 - lat1) * M_PI / 180.0;
    const double dl = (lon2 - lon1) * M_PI / 180.0;
    const double a = std::sin(dp / 2.0) * std::sin(dp / 2.0) + std::cos(p1) * std::cos(p2) * std::sin(dl / 2.0) * std::sin(dl / 2.0);
    return 2.0 * R * std::atan2(std::sqrt(a), std::sqrt(std::max(0.0, 1.0 - a)));
}

bool calculate_detection_gate(double lat, double lon, double now, int &nearest_target, double &nearest_distance) {
    const double last = last_telemetry_time.load();
    const bool fresh = telemetry_valid.load() && last > 0.0 && (now - last <= TELEMETRY_TIMEOUT);
    if (!fresh ||
        !std::isfinite(lat) ||
        !std::isfinite(lon)) {
        nearest_target = -1;
        nearest_distance = NAN;
        return false;
    }

    nearest_target = -1;
    nearest_distance = INFINITY;
    for (int i = 0; i < NUM_DETECTION_TARGETS; i++) {
        const double target_lat = DETECTION_TARGETS[i][0];
        const double target_lon = DETECTION_TARGETS[i][1];
        const double distance = haversine_distance_m(lat, lon, target_lat, target_lon);
        if (distance < nearest_distance) {
            nearest_distance = distance;
            nearest_target = i + 1;
        }
    }
    current_detection_distance.store(nearest_distance);
    return nearest_distance <= DETECTION_RADIUS_M;
}

void telemetry_udp_loop() {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        printf("[UDP] Socket telemetry gagal: %s\n", strerror(errno));
        return;
    }

    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 200000; // 200 ms agar thread dapat shutdown cepat
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(TELEMETRY_PORT);
    inet_pton(AF_INET, TELEMETRY_BIND_IP, &local.sin_addr);
    if (bind(fd, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
        printf("[UDP] Bind %s:%d gagal: %s\n", TELEMETRY_BIND_IP, TELEMETRY_PORT, strerror(errno));
        close(fd);
        return;
    }

    printf("[UDP] Menunggu telemetry npord di %s:%d\n", TELEMETRY_BIND_IP, TELEMETRY_PORT);
    char buffer[512];
    while (run) {
        ssize_t n = recvfrom(fd, buffer, sizeof(buffer) - 1, 0, nullptr, nullptr);
        if (n <= 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                continue;

            if (run)
                printf("[UDP] recvfrom gagal: %s\n", strerror(errno));
            continue;
        }

        buffer[n] = '\0';
        double lat = NAN, lon = NAN;
        // Desain A: str hanya menerima LAT/LON realtime dari npord.
        // Format paket: LAT=<latitude> LON=<longitude>
        const int parsed = std::sscanf(buffer, "LAT=%lf LON=%lf", &lat, &lon);
        if (parsed != 2 || !std::isfinite(lat) || !std::isfinite(lon)) {
            printf("[UDP] Paket telemetry tidak valid: %s\n", buffer);
            continue;
        }

        current_lat.store(lat);
        current_lon.store(lon);
        telemetry_valid.store(true);
        const double now = cv::getTickCount() / cv::getTickFrequency();
        last_telemetry_time.store(now);
        int nearest_target = -1;
        double distance = NAN;
        const bool enable = calculate_detection_gate(lat, lon, now, nearest_target, distance);
        // const double distance = current_detection_distance.load();
        const bool previous = detection_enable.exchange(enable);
        if (enable != previous) {
            printf("[DETECT] %s | TARGET=%d | DIST=%.2f m | RADIUS=%.2f m | LAT=%.7f LON=%.7f\n", enable ? "ON" : "OFF", nearest_target, distance, DETECTION_RADIUS_M, lat, lon);
        }
    }
    detection_enable.store(false);
    telemetry_valid.store(false);
    close(fd);
    printf("[UDP] Telemetry thread berhenti\n");
}

void camera_loop() {
    cv::Mat frame;
    cv::VideoCapture cap;
    int cam_index = -1;
    
    std::deque<cv::Mat> buff;
    time_t ftime, stime;
    cv::VideoWriter fwriter = create_writer("Nasional", ftime);
    
    cv::Point2f best_center;
    cv::Rect best_rect;
    cv::Rect track_roi;
    std::string target_color = "UNKNOWN";
    time_t last_det = 0;
    bool record = false;
    bool detected = false;
    bool roi_lock = false;
    int lost_count = 0;
    int detect_count = 0;
    const int LOST_LIMIT = 10;
    double smoothV = 120;
    auto last_frame_time = std::chrono::steady_clock::now();
    int fail_count = 0;
    int no_frame_count = 0;

    while (run) {
         if (!cap.isOpened()) {
            cap = open_camera(cam_index);

            if (!cap.isOpened()) {
                fail_count++;
                printf("[WARN] Tidak ada kamera (%d)\n", fail_count);
                if (fail_count > 5) {
                    printf("[RECOVERY] Reset driver kamera\n");
                    system("sudo modprobe -r uvcvideo");
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    system("sudo modprobe uvcvideo");
                    fail_count = 0;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
            fail_count = 0;
            no_frame_count = 0;
            last_frame_time =std::chrono::steady_clock::now();
        }

        if (!cap.read(frame) || frame.empty()) {
            no_frame_count++;
            printf("[WARN] Frame kosong (%d/10)\n", no_frame_count);
            if (no_frame_count > 10) {
                printf("[ERROR] Kamera hang -> reconnect\n");
                cap.release();
                no_frame_count = 0;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            continue;
        }

        no_frame_count = 0;
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds> (now - last_frame_time).count() > 2) {
            printf("[ERROR] Kamera freeze -> reconnect paksa\n");
            cap.release();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        last_frame_time = now;
        int h = frame.rows;
        int w = frame.cols;
        int cframe = w / 2;
        detected = false;
        std::string status = "SEARCH";
        double best_score = -1;
        // Detection benar-benar aktif hanya saat gate telemetry ON.
        if (!detection_enable.load()) {
            has_target = false;
            detected = false;
            roi_lock = false;
            track_roi = cv::Rect();
            lost_count = 0;
            detect_count = 0;
            target_color = "UNKNOWN";
            status = "DETECTION OFF";
        } else {
            cv::Mat gray, blur, hsv;
            // cv::medianBlur(frame, blur, 5);
            cv::medianBlur(frame, blur, 3);
            cv::GaussianBlur(blur, blur, cv::Size(5, 5), 0);
            // cv::GaussianBlur(frame, blur, cv::Size(5, 5), 0);
            // cv::GaussianBlur(frame, blur, cv::Size(7, 7), 0);

            // Color HSV
            int blh = 90;
            int bls = 75;
            int blv = 145;

            int buh = 130; // Hue Up
            int bus = 255; // Saturation Up
            int buv = 255; // Value Up

            // Oren
            int olh1 = 0;
            int ols1 = 46;
            int olv1 = 176;

            int ouh1 = 30; 
            int ous1 = 255;
            int ouv1 = 255;

            int olh2 = 0;
            int ols2 = 46;
            int olv2 = 176;

            int ouh2 = 30; 
            int ous2 = 255;
            int ouv2 = 255;
            if (pagi == true) {
                // Pagi 7 - 9
                // Biru
                blh = 90;
                bls = 75;
                blv = 145;

                buh = 105; // Hue Up
                bus = 255; // Saturation Up
                buv = 255; // Value Up

                // Oren
                olh1 = 165;
                ols1 = 46;
                olv1 = 176;

                ouh1 = 180; 
                ous1 = 255;
                ouv1 = 255;

                olh2 = 0;
                ols2 = 46;
                olv2 = 176;

                ouh2 = 30; 
                ous2 = 255;
                ouv2 = 255;
            }

            if (siang == true) {
                // Siang 9 - 12
                // Biru
                blh = 90;
                bls = 75;
                blv = 145;
    
                buh = 110; // Hue Up
                bus = 255; // Saturation Up
                buv = 255; // Value Up
    
                // Oren
                olh1 = 160;
                ols1 = 46;
                olv1 = 176;

                ouh1 = 180; 
                ous1 = 255;
                ouv1 = 255;

                olh2 = 0;
                ols2 = 46;
                olv2 = 176;

                ouh2 = 30; 
                ous2 = 255;
                ouv2 = 255;
            }

            if (sore == true) { // alias mendung
                // Sore 13 - 16
                // Biru
                blh = 90;
                bls = 75;
                blv = 145;
    
                buh = 110; // Hue Up
                bus = 255; // Saturation Up
                buv = 255; // Value Up
    
                // Oren
                olh1 = 165;
                ols1 = 46;
                olv1 = 176;

                ouh1 = 180; 
                ous1 = 255;
                ouv1 = 255;

                olh2 = 0;
                ols2 = 46;
                olv2 = 176;

                ouh2 = 30; 
                ous2 = 255;
                ouv2 = 255;
            }

            cv::Mat bmask, omask,omask1, omask2, mask;
            cv::cvtColor(blur, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, cv::Scalar(blh, bls, blv), cv::Scalar(buh, bus, buv), bmask);
            cv::inRange(hsv, cv::Scalar(olh1, ols1, olv1), cv::Scalar(ouh1, ous1, ouv1), omask1);
            cv::inRange(hsv, cv::Scalar(olh2, ols2, olv2), cv::Scalar(ouh2, ous2, ouv2), omask2);
            cv::bitwise_or(omask1, omask2, omask);
            cv::bitwise_or(bmask, omask, mask);

            // static cv::Mat kernel = cv::Mat::ones(5, 5, CV_8U);
            // static cv::Mat kernel = cv::Mat::ones(7, 7, CV_8U);
            static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

            cv::Mat proc_mask;
            cv::Point roi_offset(0,0);
            if (roi_lock) {
                cv::Rect safe_roi = track_roi & cv::Rect(0, 0, w, h);
                if (safe_roi.width > 0 && safe_roi.height > 0) {
                    proc_mask = mask(safe_roi);
                    roi_offset.x = safe_roi.x;
                    roi_offset.y = safe_roi.y;
                    cv::rectangle(frame, safe_roi, cv::Scalar(255, 255, 255), 2);
                } else {
                    roi_lock = false;
                    proc_mask = mask;
                }
            } else {
                proc_mask = mask;
            }

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(proc_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            if (roi_lock) {
                for (auto &cnt : contours) {
                    for (auto &p : cnt) {
                        p.x += roi_offset.x;
                        p.y += roi_offset.y;
                    }
                }
            }

            for (size_t idx = 0; idx < contours.size(); idx++) {
                auto cnt = contours[idx];
                double area = cv::contourArea(cnt);
                if (area < 150)
                    continue;

                if (area > w * h * 0.6)
                    continue;

                double peri = cv::arcLength(cnt, true);
                std::vector<cv::Point> approx;
                cv::approxPolyDP(cnt, approx, 0.03 * peri, true);
                double maxCos = 0;
                for (size_t i = 0; i < approx.size(); i++){
                    cv::Point p1 = approx[(i + 1) % approx.size()];
                    cv::Point p2 = approx[(i - 1 + approx.size()) % approx.size()];
                    cv::Point p0 = approx[i];
                    double cosv = std::abs(angle(p1, p2, p0));
                    maxCos = std::max(maxCos, cosv);
                }
                cv::Rect rect = cv::boundingRect(cnt);
                int rw = rect.width;
                int rh = rect.height;
                if (rw < 10 || rh < 10)
                    continue;

                double ratio = (double)std::max(rw, rh) / (std::min(rw, rh) + 1e-10);
                double extent = area / (rw * rh + 1e-10);
                std::vector<cv::Point> hull;
                cv::convexHull(cnt, hull);
                double hull_area = cv::contourArea(hull);
                double solidity = 0;
                if (hull_area > 0)
                    solidity = area / hull_area;

                double circularity = (4 * CV_PI * area) / (peri * peri + 1e-10);
                cv::Mat cmask = cv::Mat::zeros(rect.height, rect.width, CV_8UC1);
                std::vector<cv::Point> cnt_roi;
                for (auto &p : cnt)
                    cnt_roi.emplace_back(p.x - rect.x, p.y - rect.y);
                cv::drawContours(cmask, std::vector<std::vector<cv::Point>>{cnt_roi}, -1, cv::Scalar(255), -1);

                cv::Mat bmask_roi = bmask(rect);
                cv::Mat omask_roi = omask(rect);
                cv::Mat bonly, oronly;
                cv::bitwise_and(bmask_roi, cmask, bonly);
                cv::bitwise_and(omask_roi, cmask, oronly);

                int bpx = cv::countNonZero(bonly);
                int orpx = cv::countNonZero(oronly);
                double blue_ratio = static_cast<double> (bpx) / (area + 1e-10);
                double orange_ratio = static_cast<double> (orpx) / (area + 1e-10);

                std::string color = "UNKNOWN";
                double color_ratio = 0.0;
                double color_th = 0.5;

                if (blue_ratio > color_th && blue_ratio > orange_ratio) {
                    color = "BLUE";
                    color_ratio = blue_ratio;
                } else if (orange_ratio > color_th && orange_ratio > blue_ratio) {
                    color = "ORANGE";
                    color_ratio = orange_ratio;
                } else {
                    color = "UNKNOWN";
                    color_ratio = std::max(blue_ratio, orange_ratio);
                }

                bool ok = true;
                if (approx.size() < 4 || approx.size() > 6)
                    ok = false;
                else if (maxCos > 0.88)
                    ok = false;
                else if (circularity < 0.65)
                    ok = false;
                else if (extent < 0.63)
                    ok = false;
                else if (solidity < 0.60)
                    ok = false;
                else if (ratio > 1.5)
                    ok = false;
                else if (color == "UNKNOWN")
                    ok = false;

                if (!ok)
                    continue;

                cv::Moments M = cv::moments(cnt);
                if (M.m00 == 0)
                    continue;

                cv::Point2f center(M.m10 / M.m00, M.m01 / M.m00);
                double dist = std::abs(center.x - cframe);
                double score = area - (dist * 2.0);
                if (score > best_score) {
                    best_score = score;
                    best_center = center;
                    best_rect = rect;
                    target_color = color;
                    detected = true;
                }

                cv::Scalar box_color;
                if (color == "BLUE")
                    box_color = cv::Scalar(255, 0, 0);
                else if (color == "ORANGE")
                    box_color = cv::Scalar(0, 0, 255);
                else
                    box_color = cv::Scalar(0, 255, 0);

                cv::polylines(frame, approx, true, box_color, 2);
                cv::rectangle(frame, rect, box_color, 2);
            }

        if (detected) {
            has_target = true;
            lost_count = 0;
            detect_count++;
            if (detect_count > 3 && best_score > 300)
                roi_lock = true;
            cv::circle(frame, best_center, 5, cv::Scalar(0, 0, 255), -1);
            cv::rectangle(frame, best_rect, cv::Scalar(255, 0, 0), 2);
            cv::putText(frame, target_color, cv::Point(best_rect.x, best_rect.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
            int error = best_center.x - cframe;
            status = (std::abs(error) < 60) ? "LOCKED" : "TRACKING";

            int roi_size = std::max(best_rect.width, best_rect.height) * 3.5;

            int rx = best_center.x - roi_size / 2;
            int ry = best_center.y - roi_size / 2;
            rx = std::max(0, rx);
            ry = std::max(0, ry);
            if (rx + roi_size > w)
                roi_size = w - rx;

            if (ry + roi_size > h)
                roi_size = h - ry;

            cv::Rect new_roi(rx, ry, roi_size, roi_size);
            if (!roi_lock) {
                track_roi = new_roi;
            } else {
                track_roi.x = 0.8 * track_roi.x + 0.2 * new_roi.x;
                track_roi.y = 0.8 * track_roi.y + 0.2 * new_roi.y;
                track_roi.width = 0.8 * track_roi.width + 0.2 * new_roi.width;
                track_roi.height =0.8 * track_roi.height + 0.2 * new_roi.height;
            }
        } else {
            has_target = false;
            detect_count = 0;
            lost_count++;
            if (roi_lock)
                status = "LOST";
            else
                status = "SEARCH";

            if (lost_count > LOST_LIMIT){
                roi_lock = false;
                track_roi = cv::Rect();
            }
        } // end if(detected) else
    } // end else: detection_enable == true

        // Fail-safe: jika npord berhenti, detection harus mati walaupun frame kamera tetap jalan.
        const double now_telem = cv::getTickCount() / cv::getTickFrequency();
        const double last_telem = last_telemetry_time.load();
        const bool telemetry_fresh = telemetry_valid.load() && last_telem > 0.0 && (now_telem - last_telem <= TELEMETRY_TIMEOUT);
        if (!telemetry_fresh) {
            detection_enable.store(false);
            current_detection_distance.store(NAN);
        }

        cv::line(frame, cv::Point(cframe, 0), cv::Point(cframe, h), cv::Scalar(0, 0, 0), 2);

        char telem_overlay[256];
        const double lat_overlay = current_lat.load();
        const double lon_overlay = current_lon.load();
        const double dist_overlay = current_detection_distance.load();
        if (telemetry_fresh && std::isfinite(lat_overlay) && std::isfinite(lon_overlay) && std::isfinite(dist_overlay)) {
            std::snprintf(telem_overlay, sizeof(telem_overlay), "LAT:%.6f LON:%.6f D:%.1fm/R:%.1fm DET:%s", lat_overlay, lon_overlay, dist_overlay, DETECTION_RADIUS_M, detection_enable.load() ? "ON" : "OFF");
        } else {
            std::snprintf(telem_overlay, sizeof(telem_overlay), "GPS : STALE D : ---/R : %.1fm DET : OFF", DETECTION_RADIUS_M);
        }

        cv::putText(frame, status, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 0), 2);
        cv::putText(frame, telem_overlay, cv::Point(20, 65), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 0), 2);
        fps.update();
        if (fwriter.isOpened())
            fwriter.write(frame);

        buff.push_back(frame.clone());
        if (buff.size() > FPS * BUFF_SEC)
            buff.pop_front();

        if (time(0) - ftime > ROTATE_SEC) {
            fwriter.release();
            fwriter = create_writer("Nasional", ftime);
        }

        if (global_appsrc) {
            int size = frame.total() * frame.elemSize();
            GstBuffer *gst_buff = gst_buffer_new_allocate(NULL, size, NULL);
            GstMapInfo map;
            gst_buffer_map(gst_buff, &map, GST_MAP_WRITE);
            memcpy(map.data, frame.data, size);
            gst_buffer_unmap(gst_buff, &map);
            GST_BUFFER_PTS(gst_buff) = gst_util_uint64_scale(numframe, GST_SECOND, FPS);
            GST_BUFFER_DTS(gst_buff) = GST_BUFFER_PTS(gst_buff);
            GST_BUFFER_DURATION(gst_buff) = GST_SECOND / FPS;

            GstFlowReturn ret;
            g_signal_emit_by_name(global_appsrc, "push-buffer", gst_buff, &ret);
            gst_buffer_unref(gst_buff);
            numframe++;
        }
    }
    fwriter.release();
    if (global_appsrc) {
        gst_app_src_end_of_stream(GST_APP_SRC(global_appsrc));
        gst_object_unref(global_appsrc);
        global_appsrc = NULL;
    }
    
    printf("[INFO] DONE\n");
}


class UDPStreamer {
public:
    UDPStreamer(const char* destination_ip, int port) {
        snprintf(pipeline, sizeof(pipeline),
            "appsrc name=mysrc is-live=true format=time do-timestamp=true block=false "
            "caps=video/x-raw,format=BGR,width=%d,height=%d,framerate=%d/1 "
            "! videoconvert "
            "! video/x-raw,format=I420 "
            "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=1500 "
            "key-int-max=15 bframes=0 byte-stream=true "
            "! h264parse config-interval=1 "
            "! rtph264pay pt=96 config-interval=1 mtu=1200 "
            "! queue max-size-buffers=2 leaky=downstream "
            "! udpsink host=%s port=%d sync=false async=false",
            WIDTH, HEIGHT, FPS, destination_ip, port);

        GError *error = NULL;
        stream_pipeline = gst_parse_launch(pipeline, &error);
        if (!stream_pipeline) {
            printf("[ERROR] GStreamer UDP pipeline gagal dibuat: %s\n",
                   error ? error->message : "unknown error");
            if (error)
                g_error_free(error);
            return;
        }

        global_appsrc = gst_bin_get_by_name(GST_BIN(stream_pipeline), "mysrc");
        if (!global_appsrc) {
            printf("[ERROR] appsrc 'mysrc' tidak ditemukan\n");
            gst_object_unref(stream_pipeline);
            stream_pipeline = NULL;
            return;
        }

        gst_app_src_set_stream_type(GST_APP_SRC(global_appsrc), GST_APP_STREAM_TYPE_STREAM);
        gst_app_src_set_latency(GST_APP_SRC(global_appsrc), 0, 0);
        GstStateChangeReturn ret = gst_element_set_state(stream_pipeline, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            printf("[ERROR] UDP pipeline gagal masuk PLAYING\n");
            gst_object_unref(global_appsrc);
            global_appsrc = NULL;
            gst_object_unref(stream_pipeline);
            stream_pipeline = NULL;
            return;
        }

        printf("\n[INFO] UDP/RTP H.264 Stream Ready\n");
        printf("Destination : %s:%d\n", destination_ip, port);
        printf("Resolution  : %dx%d @ %d FPS\n", WIDTH, HEIGHT, FPS);
        printf("Codec       : H.264 x264 ultrafast, zerolatency\n");
        printf("Protocol    : RTP/H.264 over UDP\n");
        printf("QGC         : UDP video port %d\n\n", port);
    }
    bool isReady() const {
        return stream_pipeline != NULL && global_appsrc != NULL;
    }

    void shutdown() {
        if (!stream_pipeline)
            return;

        if (global_appsrc) {
            gst_app_src_end_of_stream(GST_APP_SRC(global_appsrc));
            gst_object_unref(global_appsrc);
            global_appsrc = NULL;
        }

        gst_element_set_state(stream_pipeline, GST_STATE_NULL);
        gst_object_unref(stream_pipeline);
        stream_pipeline = NULL;
    }
    ~UDPStreamer() {
        shutdown();
    }

private:
    GstElement *stream_pipeline = NULL;
};

int main(int argc, char *argv[]) {
    if (argc < 2 || argc > 3) {
        printf("Usage: %s <DEST_IP> [PORT]\n", argv[0]);
        printf("Example: %s 192.168.1.100 5600\n", argv[0]);
        printf("Example ZeroTier: %s 10.244.161.73 5600\n", argv[0]);
        return -1;
    }

    const char* destination_ip = argv[1];
    int port = 5600;
    if (argc == 3) {
        port = std::atoi(argv[2]);
        if (port < 1 || port > 65535) {
            printf("[ERROR] Port harus 1..65535\n");
            return -1;
        }
    }

    gst_init(&argc, &argv);
    std::signal(SIGINT, [](int) { run = false; });
    std::signal(SIGTERM, [](int) { run = false; });

    // Hanya telemetry_udp_loop yang menerima data Pixhawk dari npord.
    // str TIDAK membuka /dev/ttyS7 lagi.
    std::thread telemetry_thread(telemetry_udp_loop);
    UDPStreamer streamer(destination_ip, port);
    if (!streamer.isReady()) {
        run = false;
        telemetry_thread.join();
        return -1;
    }

    std::thread cam_thread(camera_loop);
    // Tunggu sampai SIGINT.
    while (run) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    cam_thread.join();
    streamer.shutdown();
    telemetry_thread.join();
    return 0;
}