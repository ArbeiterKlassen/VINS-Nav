/**
 * stereo_tcp_sender.cc — Go1 双目推流（TCP + JPEG），运行于相机所在 Jetson（.13）
 *
 * 背景：板载 UnitreeCameraSDK 的 UDP H.264 推流路径实测只发单目图
 * （Transmode 0/1/3 均为 928x800 单鱼眼），双目必须走本地 API + 自建传输。
 *
 * 用法:
 *   ./stereo_tcp_sender rect  [host] [port] [fps_cap] [quality]
 *   ./stereo_tcp_sender raw   [host] [port] [fps_cap] [quality]
 *   ./stereo_tcp_sender bench_rect [n]     # 只测帧率+JPEG编码，不发网络
 *   ./stereo_tcp_sender bench_raw  [n]
 *   默认: host=192.168.123.101 port=9101 fps_cap=0(不限) quality=85
 *
 * 协议（每条消息 33B 头 + JPEG）:
 *   magic "G1ST" (4) | eye u8 (0=left,1=right) | frame_id u64 | ts_us u64
 *   | width u32 | height u32 | jpeg_len u32 | jpeg bytes
 *   左右目共享同一 frame_id / ts_us（同一采集时刻）。
 *
 * 安全：只读相机；不接触运动系统；产物全在 /dev/shm/内存。
 */
#include <UnitreeCameraSDK.hpp>
#include <opencv2/opencv.hpp>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <chrono>

static const size_t HDR = 33;  // 4+1+8+8+4+4+4

static bool send_all(int fd, const char* p, size_t n) {
    while (n > 0) {
        ssize_t k = ::send(fd, p, n, MSG_NOSIGNAL);
        if (k <= 0) return false;
        p += k;
        n -= (size_t)k;
    }
    return true;
}

static int connect_to(const std::string& host, int port) {
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons((uint16_t)port);
    if (::inet_pton(AF_INET, host.c_str(), &a.sin_addr) != 1) { ::close(fd); return -1; }
    if (::connect(fd, (sockaddr*)&a, sizeof(a)) != 0) { ::close(fd); return -1; }
    return fd;
}

int main(int argc, char** argv) {
    std::string mode = (argc > 1) ? argv[1] : "rect";
    bool bench = (mode.rfind("bench", 0) == 0);
    bool use_rect = (mode.find("rect") != std::string::npos);

    std::string host = "192.168.123.101";
    int port = 9101;
    int fps_cap = 0;
    int quality = 85;
    int bench_frames = 60;

    if (bench) {
        if (argc > 2) bench_frames = atoi(argv[2]);
    } else {
        if (argc > 2) host = argv[2];
        if (argc > 3) port = atoi(argv[3]);
        if (argc > 4) fps_cap = atoi(argv[4]);
        if (argc > 5) quality = atoi(argv[5]);
    }

    UnitreeCamera cam("trans_rect_config.yaml");
    if (!cam.isOpened()) {
        printf("camera open failed\n");
        return 1;
    }
    cv::Size rs = cam.getRawFrameSize();
    printf("serial=%u pos=%u raw=%dx%d mode=%s bench=%d\n",
           (unsigned)cam.getSerialNumber(), (unsigned)cam.getPosNumber(),
           rs.width, rs.height, mode.c_str(), (int)bench);
    fflush(stdout);

    cam.startCapture();
    usleep(500000);

    int fd = -1;
    if (!bench) {
        while (fd < 0) {
            fd = connect_to(host, port);
            if (fd < 0) {
                printf("connect %s:%d failed, retry...\n", host.c_str(), port);
                fflush(stdout);
                usleep(1000000);
            }
        }
        printf("connected %s:%d\n", host.c_str(), port);
        fflush(stdout);
    }

    uint64_t frame_id = 0;
    uint64_t bytes = 0;
    int64_t frames = 0;
    auto t0 = std::chrono::steady_clock::now();
    auto tprev = t0;
    const double min_period = (fps_cap > 0) ? 1.0 / fps_cap : 0.0;

    while (true) {
        if (bench && frames >= bench_frames) break;
        auto tstart = std::chrono::steady_clock::now();

        cv::Mat l, r;
        uint64_t ts_us = 0;
        bool ok = false;
        if (use_rect) {
            cv::Mat f;
            std::chrono::microseconds ts;
            ok = cam.getRectStereoFrame(l, r, f, ts);
            ts_us = (uint64_t)ts.count();
        } else {
            std::chrono::microseconds ts;
            ok = cam.getStereoFrame(l, r, ts);
            ts_us = (uint64_t)ts.count();
        }
        if (!ok || l.empty() || r.empty()) {
            usleep(2000);
            continue;
        }

        std::vector<uchar> jl, jr;
        std::vector<int> par = {cv::IMWRITE_JPEG_QUALITY, quality};
        if (!cv::imencode(".jpg", l, jl, par) || !cv::imencode(".jpg", r, jr, par)) {
            usleep(2000);
            continue;
        }

        if (!bench) {
            char hdr[HDR];
            memcpy(hdr, "G1ST", 4);
            for (int eye = 0; eye < 2; eye++) {
                const std::vector<uchar>& j = (eye == 0) ? jl : jr;
                const cv::Mat& img = (eye == 0) ? l : r;
                hdr[4] = (char)eye;
                uint64_t fid = frame_id, tsu = ts_us;
                uint32_t w = (uint32_t)img.cols, h = (uint32_t)img.rows, len = (uint32_t)j.size();
                memcpy(hdr + 5, &fid, 8);
                memcpy(hdr + 13, &tsu, 8);
                memcpy(hdr + 21, &w, 4);
                memcpy(hdr + 25, &h, 4);
                memcpy(hdr + 29, &len, 4);
                if (!send_all(fd, hdr, HDR) || !send_all(fd, (const char*)j.data(), j.size())) {
                    printf("send failed, reconnecting...\n");
                    fflush(stdout);
                    ::close(fd);
                    fd = -1;
                    while (fd < 0) { fd = connect_to(host, port); if (fd < 0) usleep(1000000); }
                    break;  // drop this frame
                }
                bytes += HDR + j.size();
            }
        } else {
            bytes += jl.size() + jr.size();
        }

        frame_id++;
        frames++;

        if (frames % 50 == 0) {
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - tprev).count();
            printf("frames=%lld fps=%.2f avg_jpeg=%d/%d B sent=%.1f MB\n",
                   (long long)frames, 50.0 / dt, (int)jl.size(), (int)jr.size(),
                   bytes / 1e6);
            fflush(stdout);
            tprev = now;
        }

        if (min_period > 0.0) {
            double used = std::chrono::duration<double>(std::chrono::steady_clock::now() - tstart).count();
            if (used < min_period) usleep((useconds_t)((min_period - used) * 1e6));
        }
    }

    cam.stopCapture();
    if (fd >= 0) ::close(fd);
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - t0).count();
    printf("done frames=%lld in %.1fs (%.2f fps), jpeg=%.1f MB\n",
           (long long)frames, dt, frames / dt, bytes / 1e6);
    return 0;
}
