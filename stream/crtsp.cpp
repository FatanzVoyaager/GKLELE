// Streaming Only
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <opencv2/opencv.hpp>
#include <gst/app/gstappsrc.h>
#include <thread>
#include <atomic>
#include <deque>
#include <csignal>
#include <ctime>
#include <glib-unix.h>
#include <sys/stat.h>

#define WIDTH 640
#define HEIGHT 360
#define FPS 30

int camin = 0;
GstElement *global_appsrc = NULL;
GMainLoop *loop = NULL;
char pipeline[2048];
std::atomic<bool> run(true);
std::atomic<int> numframe(0);

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

    if (loop)
        g_main_loop_quit(loop);
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

void camera_loop() {
    cv::Mat frame;
    cv::VideoCapture cap;
    int cam_index = -1;
    auto last_frame_time = std::chrono::steady_clock::now();
    int fail_count = 0;
    int no_frame_count = 0;
    std::deque<cv::Mat> buff;
    time_t ftime;
    cv::VideoWriter fwriter = create_writer("Nasional", ftime);

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
            last_frame_time = std::chrono::steady_clock::now();
        }
        if (!cap.read(frame) || frame.empty()) {
            no_frame_count++;
            printf("[WARN] Frame kosong (%d/10)\n",
                no_frame_count);
            if (no_frame_count > 10) {
                printf("[ERROR] Kamera hang -> reconnect\n");
                cap.release();
                no_frame_count = 0;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            continue;
        }

        no_frame_count = 0;
        auto now =std::chrono::steady_clock::now();
        if (std::chrono::duration_cast < std::chrono::seconds > (now - last_frame_time).count() > 2){
            printf("[ERROR] Kamera freeze -> reconnect paksa\n");
            cap.release();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        last_frame_time = now;
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

            if (ret != GST_FLOW_OK) {
                global_appsrc = NULL;
            }
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

class RTSPServer {
public:
    RTSPServer(int port = 8554) {
        server = gst_rtsp_server_new();
        char port_str[10];
        sprintf(port_str, "%d", port);
        gst_rtsp_server_set_service(server, port_str);
        GstRTSPMountPoints *mounts = gst_rtsp_server_get_mount_points(server);
        GstRTSPMediaFactory *factory = gst_rtsp_media_factory_new();

        snprintf(pipeline, sizeof(pipeline),
            "( appsrc name=mysrc is-live=true format=time do-timestamp=true block=true "
            "caps=video/x-raw,format=BGR,width=%d,height=%d,framerate=%d/1 "
            "! queue leaky=downstream max-size-buffers=1 "
            "! videoconvert "
            "! video/x-raw,format=NV12 "
            "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=1200 key-int-max=15 bframes=0 "
            "! h264parse config-interval=1 "
            "! rtph264pay pt=96 name=pay0 config-interval=1 )",
            WIDTH, HEIGHT, FPS
        ); // Orange Pi 5 Pro

        gst_rtsp_media_factory_set_launch(factory, pipeline);
        gst_rtsp_media_factory_set_shared(factory, TRUE);
        gst_rtsp_mount_points_add_factory(mounts, "/live", factory);
        g_object_unref(mounts);
        g_signal_connect(factory, "media-configure", G_CALLBACK(media_configure_static), NULL);
    }

    void run(const char* ip) {
        g_object_set(server, "address", ip, NULL);
        gst_rtsp_server_attach(server, NULL);

        printf("\n[INFO] RTSP Stream Ready\n");
        printf("rtsp://%s:8554/live\n", ip);

        loop = g_main_loop_new(NULL, FALSE);
        g_main_loop_run(loop);
    }

private:
    GstRTSPServer *server;
    static void media_configure_static(GstRTSPMediaFactory *, GstRTSPMedia *media, gpointer) {
        GstElement *element = gst_rtsp_media_get_element(media);
        if (global_appsrc) {
            gst_object_unref(global_appsrc);
            global_appsrc = NULL;
        }
        global_appsrc = gst_bin_get_by_name_recurse_up(GST_BIN(element), "mysrc");
        numframe = 0;
    }
};

int main(int argc, char *argv[]) {
    if (argc != 2) {
        printf("Usage: %s <IP_ADDRESS>\n", argv[0]);
        return -1;
    }

    gst_init(&argc, &argv);
    g_unix_signal_add(SIGINT, on_sigint, NULL);
    std::thread cam_thread(camera_loop);
    RTSPServer server(8554);
    server.run(argv[1]);

    cam_thread.join();
    return 0;
}