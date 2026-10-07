// Streaming Only - UDP/RTP H.264
// Versi UDP dari crtsp.cpp

#include <gst/gst.h>
#include <opencv2/opencv.hpp>
#include <gst/app/gstappsrc.h>
#include <thread>
#include <atomic>
#include <deque>
#include <csignal>
#include <ctime>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <sys/stat.h>

#define WIDTH 640
#define HEIGHT 360
#define FPS 30

// =============================================================
// GLOBAL STREAM STATE
// =============================================================
GstElement *global_appsrc = NULL;
GstElement *stream_pipeline = NULL;
std::atomic<bool> run(true);
std::atomic<int> numframe(0);

#define BUFF_SEC 1
#define ROTATE_SEC 3600

// =============================================================
// FPS COUNTER
// =============================================================
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
// =============================================================
// SIGNAL HANDLER
// =============================================================
void on_signal(int signum) {
    if (signum == SIGINT || signum == SIGTERM) {
        printf("\n[INFO] Shutdown...\n");
        run = false;
    }
}

// =============================================================
// RECORDING
// =============================================================
cv::VideoWriter create_writer(const std::string &name, time_t &start_time) {
    struct stat st{};
    if (stat("record", &st) != 0) {
        if (mkdir("record", 0755) != 0) {
            perror("[ERROR] mkdir record");
        }
    }

    char filename[256];
    time_t now = time(0);
    std::tm local_tm{};
    localtime_r(&now, &local_tm);
    strftime(filename, sizeof(filename), ("record/" + name + "_%Y%m%d_%H%M%S.mkv").c_str(), &local_tm);
    int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    cv::VideoWriter writer(filename, fourcc, FPS,cv ::Size(WIDTH, HEIGHT));
    if (!writer.isOpened())
        printf("[ERROR] Writer gagal: %s\n", filename);
    else
        printf("[INFO] Recording: %s\n", filename);

    start_time = now;
    return writer;
}

// =============================================================
// CAMERA DISCOVERY
// =============================================================
bool device_exists(int idx) {
    std::string dev = "/dev/video" + std::to_string(idx);
    struct stat st{};
    return (stat(dev.c_str(), &st) == 0) && S_ISCHR(st.st_mode);
}

cv::VideoCapture open_camera(int &used_index) {
    for (int i = 0; i < 10; i++) {
        if (!device_exists(i))
            continue;

        cv::VideoCapture cap(i, cv::CAP_V4L2);
        if (cap.isOpened()) {
            printf("[INFO] Kamera ditemukan di /dev/video%d\n", i);
            cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
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

// =============================================================
// CAMERA LOOP
// =============================================================
void camera_loop() {
    cv::Mat frame;
    cv::VideoCapture cap;
    int cam_index = -1;
    auto last_frame_time = std::chrono::steady_clock::now();
    int fail_count = 0;
    int no_frame_count = 0;
    std::deque<cv::Mat> buff;
    time_t ftime = 0;
    cv::VideoWriter fwriter = create_writer("Nasional", ftime);
    while (run) {
        // ---------------------------------------------------------
        // CAMERA OPEN / RECOVERY
        // ---------------------------------------------------------
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
            last_frame_time = std::chrono::steady_clock::now();
        }

        // ---------------------------------------------------------
        // READ FRAME
        // ---------------------------------------------------------
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
        // ---------------------------------------------------------
        // FREEZE DETECTION
        // ---------------------------------------------------------
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(
                now - last_frame_time).count() > 2) {
            printf("[ERROR] Kamera freeze -> reconnect paksa\n");
            cap.release();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        last_frame_time = now;
        // ---------------------------------------------------------
        // OPTIONAL RESIZE SAFETY
        // ---------------------------------------------------------
        if (frame.cols != WIDTH || frame.rows != HEIGHT) {
            cv::resize(frame, frame, cv::Size(WIDTH, HEIGHT), 0, 0, cv::INTER_LINEAR);
        }

        // ---------------------------------------------------------
        // FPS + RECORDING
        // ---------------------------------------------------------
        fps.update();
        if (fwriter.isOpened())
            fwriter.write(frame);

        // Preserve the same short frame buffer as crtsp.cpp.
        buff.push_back(frame.clone());
        if (buff.size() > FPS * BUFF_SEC)
            buff.pop_front();

        // Rotate recording every ROTATE_SEC.
        if (time(0) - ftime > ROTATE_SEC) {
            fwriter.release();
            fwriter = create_writer("Nasional", ftime);
        }

        // ---------------------------------------------------------
        // PUSH FRAME TO GSTREAMER appsrc
        // ---------------------------------------------------------
        GstElement *appsrc = global_appsrc;
        if (appsrc) {
            const int size = static_cast<int>(frame.total() * frame.elemSize());
            GstBuffer *gst_buff = gst_buffer_new_allocate(NULL, size, NULL);

            if (!gst_buff) {
                printf("[ERROR] Gagal alokasi GstBuffer\n");
                continue;
            }

            GstMapInfo map{};
            if (!gst_buffer_map(gst_buff, &map, GST_MAP_WRITE)) {
                printf("[ERROR] Gagal map GstBuffer\n");
                gst_buffer_unref(gst_buff);
                continue;
            }

            memcpy(map.data, frame.data, size);
            gst_buffer_unmap(gst_buff, &map);
            const guint64 pts = gst_util_uint64_scale(static_cast<guint64>(numframe.load()), GST_SECOND, FPS);
            GST_BUFFER_PTS(gst_buff) = pts;
            GST_BUFFER_DTS(gst_buff) = pts;
            GST_BUFFER_DURATION(gst_buff) = GST_SECOND / FPS;
            GstFlowReturn ret = GST_FLOW_ERROR;
            g_signal_emit_by_name(appsrc, "push-buffer", gst_buff, &ret);
            gst_buffer_unref(gst_buff);
            if (ret == GST_FLOW_OK) {
                numframe++;
            } else {
                printf("[WARN] push-buffer gagal: %s\n",
                       gst_flow_get_name(ret));
            }
        }
    }
    cap.release();
    fwriter.release();
    printf("[INFO] Camera loop DONE\n");
}

// =============================================================
// UDP/RTP H.264 STREAMER
// =============================================================
class UDPStreamer {
public:
    UDPStreamer(const char *destination_ip, int port) {
        snprintf(pipeline, sizeof(pipeline),
            "appsrc name=mysrc is-live=true format=time do-timestamp=true block=false "
            "caps=video/x-raw,format=BGR,width=%d,height=%d,framerate=%d/1 "
            "! videoconvert "
            "! video/x-raw,format=I420 "
            "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=1200 "
            "key-int-max=15 bframes=0 byte-stream=true "
            "! h264parse config-interval=1 "
            "! rtph264pay pt=96 config-interval=1 mtu=1200 "
            "! queue max-size-buffers=2 leaky=downstream "
            "! udpsink host=%s port=%d sync=false async=false",
            WIDTH, HEIGHT, FPS, destination_ip, port);

        printf("[INFO] GStreamer pipeline:\n%s\n", pipeline);
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
            shutdown();
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
    char pipeline[2048]{};
};

// =============================================================
// MAIN
// =============================================================
int main(int argc, char *argv[]) {
    if (argc < 2 || argc > 3) {
        printf("Usage: %s <DEST_IP> [PORT]\n", argv[0]);
        printf("Example: %s 192.168.1.100 5600\n", argv[0]);
        printf("Example ZeroTier: %s 10.244.161.73 5600\n", argv[0]);
        return -1;
    }

    const char *destination_ip = argv[1];
    int port = 5600;
    if (argc == 3) {
        port = std::atoi(argv[2]);
        if (port < 1 || port > 65535) {
            printf("[ERROR] Port harus 1..65535\n");
            return -1;
        }
    }

    gst_init(&argc, &argv);
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    printf("===============================================================\n");
    printf("      CUDP - CAMERA STREAM RTP/H.264 UDP\n");
    printf("===============================================================\n");
    printf("Resolution : %dx%d\n", WIDTH, HEIGHT);
    printf("FPS        : %d\n", FPS);
    printf("Destination: %s:%d\n", destination_ip, port);
    printf("===============================================================\n");
    UDPStreamer streamer(destination_ip, port);
    if (!streamer.isReady()) {
        run = false;
        return -1;
    }

    std::thread cam_thread(camera_loop);
    while (run) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    cam_thread.join();
    streamer.shutdown();
    printf("[INFO] Program selesai.\n");
    return 0;
}
