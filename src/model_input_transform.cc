#include "model_input_transform.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace {

void matmul3(const float *a, const float *b, float *out)
{
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < 3; ++k)
                sum += a[r * 3 + k] * b[k * 3 + c];
            out[r * 3 + c] = sum;
        }
    }
}

void matmul34(const float *a3, const float *b34, float *out34)
{
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < 3; ++k)
                sum += a3[r * 3 + k] * b34[k * 4 + c];
            out34[r * 4 + c] = sum;
        }
    }
}

void rotation_from_rpy(float roll, float pitch, float yaw, float *rot)
{
    const float cr = std::cos(roll);
    const float sr = std::sin(roll);
    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);
    const float cy = std::cos(yaw);
    const float sy = std::sin(yaw);

    const float rx[9] = {
        1.0f, 0.0f, 0.0f,
        0.0f, cr, -sr,
        0.0f, sr, cr,
    };
    const float ry[9] = {
        cp, 0.0f, sp,
        0.0f, 1.0f, 0.0f,
        -sp, 0.0f, cp,
    };
    const float rz[9] = {
        cy, -sy, 0.0f,
        sy, cy, 0.0f,
        0.0f, 0.0f, 1.0f,
    };

    float tmp[9];
    matmul3(ry, rx, tmp);
    matmul3(rz, tmp, rot);
}

void transform_scale_buffer(const float *in, float scale, float *out)
{
    const float transform_out[9] = {
        1.0f / scale, 0.0f, 0.5f,
        0.0f, 1.0f / scale, 0.5f,
        0.0f, 0.0f, 1.0f,
    };
    const float transform_in[9] = {
        scale, 0.0f, -0.5f * scale,
        0.0f, scale, -0.5f * scale,
        0.0f, 0.0f, 1.0f,
    };

    float tmp[9];
    matmul3(in, transform_out, tmp);
    matmul3(transform_in, tmp, out);
}

} // namespace

ModelInputTransform::ModelInputTransform(const AppConfig &config)
    : fx_(config.input_warp_fx),
      fy_(config.input_warp_fy),
      cx_(config.input_warp_cx),
      cy_(config.input_warp_cy),
      height_(config.input_warp_height),
      roll_(config.input_warp_roll),
      pitch_(config.input_warp_pitch),
      yaw_(config.input_warp_yaw)
{
}

void ModelInputTransform::set_calibration(float roll, float pitch, float yaw)
{
    constexpr float kEpsilon = 1e-7f;
    if (std::fabs(roll_ - roll) < kEpsilon &&
        std::fabs(pitch_ - pitch) < kEpsilon &&
        std::fabs(yaw_ - yaw) < kEpsilon) {
        return;
    }

    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
    map_valid_ = false;
}

uint8_t ModelInputTransform::sample_luma(const uint8_t *base, const BilinearSample &sample)
{
    int sum = 0;
    sum += static_cast<int>(base[sample.offset[0]]) * sample.weight[0];
    sum += static_cast<int>(base[sample.offset[1]]) * sample.weight[1];
    sum += static_cast<int>(base[sample.offset[2]]) * sample.weight[2];
    sum += static_cast<int>(base[sample.offset[3]]) * sample.weight[3];
    const int rounded = (sum + (kWeightScale / 2)) >> kWeightBits;
    return static_cast<uint8_t>(std::min(255, std::max(0, rounded)));
}

uint8_t ModelInputTransform::sample_chroma(const uint8_t *base, const BilinearSample &sample, int channel)
{
    int sum = 0;
    sum += static_cast<int>(base[sample.offset[0] + channel]) * sample.weight[0];
    sum += static_cast<int>(base[sample.offset[1] + channel]) * sample.weight[1];
    sum += static_cast<int>(base[sample.offset[2] + channel]) * sample.weight[2];
    sum += static_cast<int>(base[sample.offset[3] + channel]) * sample.weight[3];
    const int rounded = (sum + (kWeightScale / 2)) >> kWeightBits;
    return static_cast<uint8_t>(std::min(255, std::max(0, rounded)));
}

void ModelInputTransform::build_projection(float *projection) const
{
    // Same medmodel inverse used by openpilot modeld.update_calibration().
    const float ground_from_medmodel_frame[9] = {
        0.00000000e+00f, 0.00000000e+00f, 1.00000000e+00f,
       -1.09890110e-03f, 0.00000000e+00f, 2.81318681e-01f,
       -1.84808520e-20f, 9.00738606e-04f, -4.28751576e-02f,
    };

    const float k[9] = {
        fx_, 0.0f, cx_,
        0.0f, fy_, cy_,
        0.0f, 0.0f, 1.0f,
    };

    float rot[9];
    rotation_from_rpy(roll_, pitch_, yaw_, rot);

    float device_from_road[9];
    for (int row = 0; row < 3; ++row) {
        device_from_road[row * 3 + 0] = rot[row * 3 + 0];
        device_from_road[row * 3 + 1] = -rot[row * 3 + 1];
        device_from_road[row * 3 + 2] = -rot[row * 3 + 2];
    }

    // view_from_device = [[0,1,0],[0,0,1],[1,0,0]]
    float view_from_road[9];
    for (int col = 0; col < 3; ++col) {
        view_from_road[0 * 3 + col] = device_from_road[1 * 3 + col];
        view_from_road[1 * 3 + col] = device_from_road[2 * 3 + col];
        view_from_road[2 * 3 + col] = device_from_road[0 * 3 + col];
    }

    float extrinsic[12] = {
        view_from_road[0], view_from_road[1], view_from_road[2], 0.0f,
        view_from_road[3], view_from_road[4], view_from_road[5], height_,
        view_from_road[6], view_from_road[7], view_from_road[8], 0.0f,
    };

    float camera_frame_from_road[12];
    matmul34(k, extrinsic, camera_frame_from_road);

    float camera_frame_from_ground[9];
    for (int row = 0; row < 3; ++row) {
        camera_frame_from_ground[row * 3 + 0] = camera_frame_from_road[row * 4 + 0];
        camera_frame_from_ground[row * 3 + 1] = camera_frame_from_road[row * 4 + 1];
        camera_frame_from_ground[row * 3 + 2] = camera_frame_from_road[row * 4 + 3];
    }

    matmul3(camera_frame_from_ground, ground_from_medmodel_frame, projection);
}

void ModelInputTransform::build_sample_map(const float *projection, int src_w, int src_h,
                                           int dst_w, int dst_h, int src_stride_pixels,
                                           int bytes_per_pixel, std::vector<BilinearSample> &map) const
{
    map.resize(static_cast<size_t>(dst_w) * dst_h);
    for (int y = 0; y < dst_h; ++y) {
        for (int x = 0; x < dst_w; ++x) {
            BilinearSample sample;
            const float x0 = projection[0] * x + projection[1] * y + projection[2];
            const float y0 = projection[3] * x + projection[4] * y + projection[5];
            const float w0 = projection[6] * x + projection[7] * y + projection[8];
            if (std::fabs(w0) > 1e-6f) {
                const float sx = x0 / w0;
                const float sy = y0 / w0;
                const int ix = static_cast<int>(std::floor(sx));
                const int iy = static_cast<int>(std::floor(sy));
                const float ax = sx - ix;
                const float ay = sy - iy;
                const float weights_f[4] = {
                    (1.0f - ax) * (1.0f - ay),
                    ax * (1.0f - ay),
                    (1.0f - ax) * ay,
                    ax * ay,
                };
                const int xs[4] = {ix, ix + 1, ix, ix + 1};
                const int ys[4] = {iy, iy, iy + 1, iy + 1};
                for (int i = 0; i < 4; ++i) {
                    if (xs[i] >= 0 && xs[i] < src_w && ys[i] >= 0 && ys[i] < src_h) {
                        sample.offset[i] = static_cast<uint32_t>((ys[i] * src_stride_pixels + xs[i]) * bytes_per_pixel);
                        sample.weight[i] = static_cast<uint16_t>(
                            std::max(0.0f, std::min(static_cast<float>(kWeightScale), std::round(weights_f[i] * kWeightScale))));
                    }
                }
            }
            map[static_cast<size_t>(y) * dst_w + x] = sample;
        }
    }
}

void ModelInputTransform::rebuild_maps(int src_w, int src_h)
{
    if (src_w <= 0 || src_h <= 0 || (src_w & 1) || (src_h & 1))
        throw std::runtime_error("input warp requires positive even NV12 dimensions");

    float projection_y[9];
    float projection_uv[9];
    build_projection(projection_y);
    transform_scale_buffer(projection_y, 0.5f, projection_uv);

    build_sample_map(projection_y, src_w, src_h, kModelW, kModelH, src_w, 1, y_map_);
    build_sample_map(projection_uv, src_w / 2, src_h / 2, kHalfW, kHalfH, src_w / 2, 2, uv_map_);

    map_src_w_ = src_w;
    map_src_h_ = src_h;
    map_valid_ = true;

    std::fprintf(stderr,
                 "input warp=on source=%dx%d intrinsics=(fx=%.2f fy=%.2f cx=%.2f cy=%.2f) "
                 "rpy_deg=(%.3f %.3f %.3f)\n",
                 src_w, src_h, fx_, fy_, cx_, cy_,
                 rad_to_deg(roll_), rad_to_deg(pitch_), rad_to_deg(yaw_));
}

void ModelInputTransform::nv12_to_yuv6_warped(const uint8_t *nv12, int src_w, int src_h, std::vector<float> &out)
{
    if (!map_valid_ || src_w != map_src_w_ || src_h != map_src_h_)
        rebuild_maps(src_w, src_h);

    const uint8_t *y_src = nv12;
    const uint8_t *uv_src = nv12 + src_w * src_h;
    const int plane_size = kHalfW * kHalfH;

    float *y00_plane = out.data();
    float *y10_plane = y00_plane + plane_size;
    float *y01_plane = y10_plane + plane_size;
    float *y11_plane = y01_plane + plane_size;
    float *u_plane = y11_plane + plane_size;
    float *v_plane = u_plane + plane_size;

    for (int y2 = 0; y2 < kHalfH; ++y2) {
        for (int x2 = 0; x2 < kHalfW; ++x2) {
            const int ox = x2 * 2;
            const int oy = y2 * 2;
            const int dst_idx = y2 * kHalfW + x2;
            const size_t y_idx00 = static_cast<size_t>(oy) * kModelW + ox;
            const size_t y_idx01 = y_idx00 + 1;
            const size_t y_idx10 = y_idx00 + kModelW;
            const size_t y_idx11 = y_idx10 + 1;
            const size_t uv_idx = static_cast<size_t>(y2) * kHalfW + x2;

            y00_plane[dst_idx] = static_cast<float>(sample_luma(y_src, y_map_[y_idx00]));
            y10_plane[dst_idx] = static_cast<float>(sample_luma(y_src, y_map_[y_idx10]));
            y01_plane[dst_idx] = static_cast<float>(sample_luma(y_src, y_map_[y_idx01]));
            y11_plane[dst_idx] = static_cast<float>(sample_luma(y_src, y_map_[y_idx11]));
            u_plane[dst_idx] = static_cast<float>(sample_chroma(uv_src, uv_map_[uv_idx], 0));
            v_plane[dst_idx] = static_cast<float>(sample_chroma(uv_src, uv_map_[uv_idx], 1));
        }
    }
}
