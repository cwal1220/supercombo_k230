#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <vector>

static constexpr int kModelW = 512;
static constexpr int kModelH = 256;
static constexpr int kHalfW = kModelW / 2;
static constexpr int kHalfH = kModelH / 2;

static uint64_t now_ns()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

static void nv12_to_yuv6_float(const uint8_t *nv12, float *out)
{
    const uint8_t *y_plane = nv12;
    const uint8_t *uv_plane = nv12 + kModelW * kModelH;
    const int plane_size = kHalfW * kHalfH;
    float *y00_plane = out;
    float *y01_plane = y00_plane + plane_size;
    float *y10_plane = y01_plane + plane_size;
    float *y11_plane = y10_plane + plane_size;
    float *u_plane = y11_plane + plane_size;
    float *v_plane = u_plane + plane_size;

    for (int y2 = 0; y2 < kHalfH; ++y2) {
        const uint8_t *y0 = y_plane + (y2 * 2) * kModelW;
        const uint8_t *y1 = y0 + kModelW;
        const uint8_t *uv = uv_plane + y2 * kModelW;
        float *dst_y00 = y00_plane + y2 * kHalfW;
        float *dst_y01 = y01_plane + y2 * kHalfW;
        float *dst_y10 = y10_plane + y2 * kHalfW;
        float *dst_y11 = y11_plane + y2 * kHalfW;
        float *dst_u = u_plane + y2 * kHalfW;
        float *dst_v = v_plane + y2 * kHalfW;

        for (int x2 = 0; x2 < kHalfW; ++x2) {
            dst_y00[x2] = static_cast<float>(y0[0]);
            dst_y01[x2] = static_cast<float>(y0[1]);
            dst_y10[x2] = static_cast<float>(y1[0]);
            dst_y11[x2] = static_cast<float>(y1[1]);
            dst_u[x2] = static_cast<float>(uv[0]);
            dst_v[x2] = static_cast<float>(uv[1]);
            y0 += 2;
            y1 += 2;
            uv += 2;
        }
    }
}

int main()
{
    std::vector<uint8_t> nv12(kModelW * kModelH * 3 / 2, 127);
    std::vector<float> yuv6(6 * kHalfW * kHalfH, 0.0f);

    for (int i = 0; i < 10; ++i)
        nv12_to_yuv6_float(nv12.data(), yuv6.data());

    const int runs = 1000;
    const uint64_t t0 = now_ns();
    for (int i = 0; i < runs; ++i)
        nv12_to_yuv6_float(nv12.data(), yuv6.data());
    const uint64_t t1 = now_ns();

    float checksum = 0.0f;
    for (size_t i = 0; i < yuv6.size(); i += 4096)
        checksum += yuv6[i];

    std::printf("nv12->yuv6_float avg_ms=%.3f runs=%d checksum=%.1f\n",
                (t1 - t0) / 1000000.0 / runs, runs, checksum);
    return 0;
}
