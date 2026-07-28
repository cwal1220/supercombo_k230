#include "app_config.h"
#include "model_input_transform.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace {

constexpr int kModelW = 512;
constexpr int kModelH = 256;
constexpr int kHalfW = kModelW / 2;
constexpr int kHalfH = kModelH / 2;
constexpr int kPlaneSize = kHalfW * kHalfH;
constexpr int kYuv6Floats = 6 * kPlaneSize;

uint64_t now_ns()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

void fill_nv12(std::vector<uint8_t> &nv12, int width, int height)
{
    uint8_t *y_plane = nv12.data();
    uint8_t *uv_plane = y_plane + width * height;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x)
            y_plane[y * width + x] = static_cast<uint8_t>((x * 3 + y * 5 + (x * y) / 17) & 0xff);
    }
    for (int y = 0; y < height / 2; ++y) {
        for (int x = 0; x < width / 2; ++x) {
            uv_plane[y * width + x * 2] = static_cast<uint8_t>((64 + x * 2 + y * 3) & 0xff);
            uv_plane[y * width + x * 2 + 1] = static_cast<uint8_t>((192 + x * 5 + y) & 0xff);
        }
    }
}

void nv12_to_yuv6_direct_openpilot_order(const uint8_t *nv12, float *out)
{
    const uint8_t *y_plane = nv12;
    const uint8_t *uv_plane = nv12 + kModelW * kModelH;
    float *y00_plane = out;
    float *y10_plane = y00_plane + kPlaneSize;
    float *y01_plane = y10_plane + kPlaneSize;
    float *y11_plane = y01_plane + kPlaneSize;
    float *u_plane = y11_plane + kPlaneSize;
    float *v_plane = u_plane + kPlaneSize;

    for (int y2 = 0; y2 < kHalfH; ++y2) {
        const uint8_t *y0 = y_plane + (y2 * 2) * kModelW;
        const uint8_t *y1 = y0 + kModelW;
        const uint8_t *uv = uv_plane + y2 * kModelW;
        float *dst_y00 = y00_plane + y2 * kHalfW;
        float *dst_y10 = y10_plane + y2 * kHalfW;
        float *dst_y01 = y01_plane + y2 * kHalfW;
        float *dst_y11 = y11_plane + y2 * kHalfW;
        float *dst_u = u_plane + y2 * kHalfW;
        float *dst_v = v_plane + y2 * kHalfW;

        for (int x2 = 0; x2 < kHalfW; ++x2) {
            dst_y00[x2] = static_cast<float>(y0[0]);
            dst_y10[x2] = static_cast<float>(y1[0]);
            dst_y01[x2] = static_cast<float>(y0[1]);
            dst_y11[x2] = static_cast<float>(y1[1]);
            dst_u[x2] = static_cast<float>(uv[0]);
            dst_v[x2] = static_cast<float>(uv[1]);
            y0 += 2;
            y1 += 2;
            uv += 2;
        }
    }
}

double checksum(const std::vector<float> &yuv6)
{
    double sum = 0.0;
    for (size_t i = 0; i < yuv6.size(); i += 1024)
        sum += yuv6[i];
    return sum;
}

double max_abs_diff(const std::vector<float> &a, const std::vector<float> &b)
{
    double max_diff = 0.0;
    for (size_t i = 0; i < a.size(); ++i)
        max_diff = std::max(max_diff, static_cast<double>(std::abs(a[i] - b[i])));
    return max_diff;
}

template <typename Fn>
double bench_ms(int runs, Fn fn)
{
    const uint64_t t0 = now_ns();
    for (int i = 0; i < runs; ++i)
        fn();
    const uint64_t t1 = now_ns();
    return (t1 - t0) / 1000000.0 / runs;
}

} // namespace

int main(int argc, char **argv)
{
    const int runs = argc > 1 ? std::max(1, std::atoi(argv[1])) : 2000;
    std::vector<uint8_t> nv12(kModelW * kModelH * 3 / 2);
    std::vector<float> direct(kYuv6Floats, 0.0f);
    std::vector<float> warped(kYuv6Floats, 0.0f);
    std::vector<float> pitched(kYuv6Floats, 0.0f);
    fill_nv12(nv12, kModelW, kModelH);

    AppConfig identity_config;
    ModelInputTransform identity_warp(identity_config);

    AppConfig pitch_config = identity_config;
    pitch_config.input_warp_pitch = deg_to_rad(1.5f);
    ModelInputTransform pitch_warp(pitch_config);

    nv12_to_yuv6_direct_openpilot_order(nv12.data(), direct.data());

    const uint64_t first_identity_t0 = now_ns();
    identity_warp.nv12_to_yuv6_warped(nv12.data(), kModelW, kModelH, warped);
    const uint64_t first_identity_t1 = now_ns();
    pitch_warp.nv12_to_yuv6_warped(nv12.data(), kModelW, kModelH, pitched);

    for (int i = 0; i < 20; ++i) {
        nv12_to_yuv6_direct_openpilot_order(nv12.data(), direct.data());
        identity_warp.nv12_to_yuv6_warped(nv12.data(), kModelW, kModelH, warped);
        pitch_warp.nv12_to_yuv6_warped(nv12.data(), kModelW, kModelH, pitched);
    }

    const double direct_ms = bench_ms(runs, [&] {
        nv12_to_yuv6_direct_openpilot_order(nv12.data(), direct.data());
    });
    const double identity_ms = bench_ms(runs, [&] {
        identity_warp.nv12_to_yuv6_warped(nv12.data(), kModelW, kModelH, warped);
    });
    const double pitch_ms = bench_ms(runs, [&] {
        pitch_warp.nv12_to_yuv6_warped(nv12.data(), kModelW, kModelH, pitched);
    });

    std::printf("runs=%d frame=512x256\n", runs);
    std::printf("first_identity_call_ms=%.3f  # includes LUT build\n",
                (first_identity_t1 - first_identity_t0) / 1000000.0);
    std::printf("direct_openpilot_order_ms=%.3f checksum=%.1f\n", direct_ms, checksum(direct));
    std::printf("warp_identity_ms=%.3f checksum=%.1f max_abs_vs_direct=%.1f overhead_ms=%.3f overhead_x=%.2f\n",
                identity_ms, checksum(warped), max_abs_diff(direct, warped),
                identity_ms - direct_ms, identity_ms / direct_ms);
    std::printf("warp_pitch_1p5deg_ms=%.3f checksum=%.1f overhead_ms=%.3f overhead_x=%.2f\n",
                pitch_ms, checksum(pitched), pitch_ms - direct_ms, pitch_ms / direct_ms);

    constexpr int kSourceW = 640;
    constexpr int kSourceH = 360;
    std::vector<uint8_t> source_nv12(kSourceW * kSourceH * 3 / 2);
    std::vector<float> med_scalar(kYuv6Floats, 0.0f);
    std::vector<float> med_rvv(kYuv6Floats, 0.0f);
    std::vector<float> sbig_scalar(kYuv6Floats, 0.0f);
    std::vector<float> sbig_rvv(kYuv6Floats, 0.0f);
    fill_nv12(source_nv12, kSourceW, kSourceH);

    AppConfig source_config;
    source_config.input_warp_pitch = deg_to_rad(-0.75f);
    source_config.input_warp_yaw = deg_to_rad(1.1f);
    ModelInputTransform med(source_config, ModelFrame::MedModel);
    ModelInputTransform sbig(source_config, ModelFrame::SmallBigModel);
    med.nv12_to_yuv6_warped_scalar(
        source_nv12.data(), kSourceW, kSourceH, med_scalar.data());
    med.nv12_to_yuv6_warped_rvv(
        source_nv12.data(), kSourceW, kSourceH, med_rvv.data());
    sbig.nv12_to_yuv6_warped_scalar(
        source_nv12.data(), kSourceW, kSourceH, sbig_scalar.data());
    sbig.nv12_to_yuv6_warped_rvv(
        source_nv12.data(), kSourceW, kSourceH, sbig_rvv.data());

    const bool med_exact = std::memcmp(
        med_scalar.data(), med_rvv.data(), med_scalar.size() * sizeof(float)) == 0;
    const bool sbig_exact = std::memcmp(
        sbig_scalar.data(), sbig_rvv.data(), sbig_scalar.size() * sizeof(float)) == 0;
    const double med_scalar_ms = bench_ms(runs, [&] {
        med.nv12_to_yuv6_warped_scalar(
            source_nv12.data(), kSourceW, kSourceH, med_scalar.data());
    });
    const double med_rvv_ms = bench_ms(runs, [&] {
        med.nv12_to_yuv6_warped_rvv(
            source_nv12.data(), kSourceW, kSourceH, med_rvv.data());
    });
    const double sbig_scalar_ms = bench_ms(runs, [&] {
        sbig.nv12_to_yuv6_warped_scalar(
            source_nv12.data(), kSourceW, kSourceH, sbig_scalar.data());
    });
    const double sbig_rvv_ms = bench_ms(runs, [&] {
        sbig.nv12_to_yuv6_warped_rvv(
            source_nv12.data(), kSourceW, kSourceH, sbig_rvv.data());
    });

    std::printf("rvv_available=%d med_bit_exact=%d sbig_bit_exact=%d\n",
                ModelInputTransform::rvv_available() ? 1 : 0,
                med_exact ? 1 : 0, sbig_exact ? 1 : 0);
    std::printf("med_scalar_ms=%.3f med_rvv_ms=%.3f speedup=%.2fx\n",
                med_scalar_ms, med_rvv_ms, med_scalar_ms / med_rvv_ms);
    std::printf("sbig_scalar_ms=%.3f sbig_rvv_ms=%.3f speedup=%.2fx\n",
                sbig_scalar_ms, sbig_rvv_ms, sbig_scalar_ms / sbig_rvv_ms);
    return med_exact && sbig_exact ? 0 : 1;
}
