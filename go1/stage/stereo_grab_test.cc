/**
 * stereo_grab_test.cc — 本地双目抓取验证（不推流，只落 /dev/shm）
 *
 * 背景：板载 UnitreeCameraSDK 的 UDP H.264 推流路径（example_putImagetrans）
 * 实测只发送单目图（Transmode 0/1/3 均为 928x800 单鱼眼），双目需走本地 API。
 *
 * 用法: ./stereo_grab_test rect|raw [n]
 * 产物: /dev/shm/stereo_test/{rect|raw}_{left,right}_NN.png
 */
#include <UnitreeCameraSDK.hpp>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>

int main(int argc, char** argv) {
    const char* mode = (argc > 1) ? argv[1] : "rect";
    int want = (argc > 2) ? atoi(argv[2]) : 4;

    UnitreeCamera cam("trans_rect_config.yaml");
    if (!cam.isOpened()) {
        printf("camera open failed\n");
        return 1;
    }
    cv::Size rs = cam.getRawFrameSize();
    printf("serial=%u pos=%u raw=%dx%d\n",
           (unsigned)cam.getSerialNumber(), (unsigned)cam.getPosNumber(),
           rs.width, rs.height);

    cam.startCapture();       // no udp, no shared memory
    usleep(500000);

    int got = 0;
    for (int i = 0; i < 300 && got < want; i++) {
        char p[160];
        if (strcmp(mode, "rect") == 0) {
            cv::Mat l, r, f;
            std::chrono::microseconds ts;
            if (cam.getRectStereoFrame(l, r, f, ts)) {
                snprintf(p, sizeof p, "/dev/shm/stereo_test/rect_left_%02d.png", got);
                cv::imwrite(p, l);
                snprintf(p, sizeof p, "/dev/shm/stereo_test/rect_right_%02d.png", got);
                cv::imwrite(p, r);
                printf("rect[%d] L=%dx%d R=%dx%d feim=%dx%d ts=%lld us\n",
                       got, l.cols, l.rows, r.cols, r.rows, f.cols, f.rows,
                       (long long)ts.count());
                got++;
            }
        } else {
            cv::Mat l, r;
            std::chrono::microseconds ts;
            if (cam.getStereoFrame(l, r, ts)) {
                snprintf(p, sizeof p, "/dev/shm/stereo_test/raw_left_%02d.png", got);
                cv::imwrite(p, l);
                snprintf(p, sizeof p, "/dev/shm/stereo_test/raw_right_%02d.png", got);
                cv::imwrite(p, r);
                printf("raw[%d] L=%dx%d R=%dx%d ts=%lld us\n",
                       got, l.cols, l.rows, r.cols, r.rows,
                       (long long)ts.count());
                got++;
            }
        }
        usleep(80000);
    }

    cam.stopCapture();
    printf("done, got=%d\n", got);
    return got > 0 ? 0 : 2;
}
