#include "mmz.h"
#include "v4l2-drm.h"

#include <linux/videodev2.h>

#include <chrono>
#include <cstdio>

static uint64_t now_ns()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

int main()
{
    v4l2_drm_context ctx;
    v4l2_drm_default_context(&ctx);
    ctx.device = kd_mpi_get_vvcam_video00() + 1;
    ctx.display = false;
    ctx.width = 512;
    ctx.height = 256;
    ctx.video_format = V4L2_PIX_FMT_NV12;
    ctx.buffer_num = 3;

    if (v4l2_drm_setup(&ctx, 1, nullptr)) {
        std::fprintf(stderr, "v4l2_drm_setup NV12 512x256 failed\n");
        return 1;
    }
    if (v4l2_drm_start(&ctx)) {
        std::fprintf(stderr, "v4l2_drm_start failed\n");
        return 1;
    }

    for (int i = 0; i < 5; ++i) {
        if (v4l2_drm_dump(&ctx, 1000)) {
            std::perror("warmup v4l2_drm_dump");
            return 1;
        }
        v4l2_drm_dump_release(&ctx);
    }

    const int frames = 60;
    const uint64_t t0 = now_ns();
    for (int i = 0; i < frames; ++i) {
        if (v4l2_drm_dump(&ctx, 1000)) {
            std::perror("v4l2_drm_dump");
            return 1;
        }
        v4l2_drm_dump_release(&ctx);
    }
    const uint64_t t1 = now_ns();

    std::printf("capture NV12 512x256 avg_ms=%.3f fps=%.2f frame_count=%u\n",
                (t1 - t0) / 1000000.0 / frames,
                frames * 1000000000.0 / (t1 - t0),
                ctx.frame_count);

    v4l2_drm_stop(&ctx);
    return 0;
}
