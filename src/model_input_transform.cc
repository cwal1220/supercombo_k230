#include "model_input_transform.h"

#include "projection.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

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

bool scalar_warp_forced()
{
    static const bool forced = [] {
        const char *value = std::getenv("SUPERCOMBO_WARP_SCALAR");
        return value && value[0] != '\0' && std::strcmp(value, "0") != 0;
    }();
    return forced;
}

} // namespace

void ModelInputTransform::SampleMap::resize(size_t size)
{
    offset.resize(size);
    x_step.resize(size);
    y_step.resize(size);
    for (auto &weights : weight)
        weights.resize(size);
}

ModelInputTransform::ModelInputTransform(const AppConfig &config, ModelFrame model_frame)
    : fx_(config.input_warp_fx),
      fy_(config.input_warp_fy),
      cx_(config.input_warp_cx),
      cy_(config.input_warp_cy),
      height_(config.input_warp_height),
      roll_(config.manual_roll),
      pitch_(config.manual_pitch),
      yaw_(config.manual_yaw),
      model_frame_(model_frame)
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

uint8_t ModelInputTransform::sample(const uint8_t *base, const SampleMap &map,
                                    size_t index, int channel)
{
    const uint32_t offset = map.offset[index] + static_cast<uint32_t>(channel);
    const uint32_t x_step = map.x_step[index];
    const uint32_t y_step = map.y_step[index];
    uint32_t sum = 0;
    sum += static_cast<uint32_t>(base[offset]) * map.weight[0][index];
    sum += static_cast<uint32_t>(base[offset + x_step]) * map.weight[1][index];
    sum += static_cast<uint32_t>(base[offset + y_step]) * map.weight[2][index];
    sum += static_cast<uint32_t>(base[offset + y_step + x_step]) * map.weight[3][index];
    const int rounded = (sum + (kWeightScale / 2)) >> kWeightBits;
    return static_cast<uint8_t>(std::min(255, std::max(0, rounded)));
}

void ModelInputTransform::projection_matrix(float *projection) const
{
    // Same model-frame inverses used by openpilot modeld.update_calibration().
    const float ground_from_medmodel_frame[9] = {
        0.00000000e+00f, 0.00000000e+00f, 1.00000000e+00f,
       -1.09890110e-03f, 0.00000000e+00f, 2.81318681e-01f,
       -1.84808520e-20f, 9.00738606e-04f, -4.28751576e-02f,
    };
    const float ground_from_sbigmodel_frame[9] = {
        0.00000000e+00f,  7.31372216e-19f,  1.00000000e+00f,
       -2.19780220e-03f,  4.11497335e-19f,  5.62637363e-01f,
       -5.46146580e-20f,  1.80147721e-03f, -2.73464241e-01f,
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

    const float *ground_from_model_frame = model_frame_ == ModelFrame::SmallBigModel
        ? ground_from_sbigmodel_frame
        : ground_from_medmodel_frame;
    matmul3(camera_frame_from_ground, ground_from_model_frame, projection);
}

void ModelInputTransform::build_sample_map(const float *projection, int src_w, int src_h,
                                           int dst_w, int dst_h, int src_stride_pixels,
                                           int bytes_per_pixel, int dst_scale,
                                           int dst_x_offset, int dst_y_offset,
                                           SampleMap &map) const
{
    const size_t sample_count = static_cast<size_t>(dst_w) * dst_h;
    map.resize(sample_count);
    for (int y = 0; y < dst_h; ++y) {
        for (int x = 0; x < dst_w; ++x) {
            const int dst_x = x * dst_scale + dst_x_offset;
            const int dst_y = y * dst_scale + dst_y_offset;
            const float x0 = projection[0] * dst_x + projection[1] * dst_y + projection[2];
            const float y0 = projection[3] * dst_x + projection[4] * dst_y + projection[5];
            const float w0 = projection[6] * dst_x + projection[7] * dst_y + projection[8];
            const size_t index = static_cast<size_t>(y) * dst_w + x;
            int ix = -2;
            int iy = -2;
            if (std::fabs(w0) > 1e-6f) {
                const float sx = x0 / w0;
                const float sy = y0 / w0;
                ix = static_cast<int>(std::floor(sx));
                iy = static_cast<int>(std::floor(sy));
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
                        map.weight[i][index] = static_cast<uint16_t>(
                            std::max(0.0f, std::min(static_cast<float>(kWeightScale), std::round(weights_f[i] * kWeightScale))));
                    } else {
                        map.weight[i][index] = 0;
                    }
                }
            } else {
                for (auto &weights : map.weight)
                    weights[index] = 0;
            }

            const int base_x = std::max(0, std::min(src_w - 1, ix));
            const int base_y = std::max(0, std::min(src_h - 1, iy));
            map.offset[index] = static_cast<uint32_t>(
                (base_y * src_stride_pixels + base_x) * bytes_per_pixel);
            map.x_step[index] = ix >= 0 && ix + 1 < src_w
                ? static_cast<uint16_t>(bytes_per_pixel)
                : 0;
            map.y_step[index] = iy >= 0 && iy + 1 < src_h
                ? static_cast<uint16_t>(src_stride_pixels * bytes_per_pixel)
                : 0;
        }
    }
}

void ModelInputTransform::rebuild_maps(int src_w, int src_h)
{
    if (src_w <= 0 || src_h <= 0 || (src_w & 1) || (src_h & 1))
        throw std::runtime_error("input warp requires positive even NV12 dimensions");

    float projection_y[9];
    float projection_uv[9];
    projection_matrix(projection_y);
    transform_scale_buffer(projection_y, 0.5f, projection_uv);

    constexpr int x_offsets[4] = {0, 0, 1, 1};
    constexpr int y_offsets[4] = {0, 1, 0, 1};
    for (int plane = 0; plane < 4; ++plane) {
        build_sample_map(projection_y, src_w, src_h, kHalfW, kHalfH, src_w, 1,
                         2, x_offsets[plane], y_offsets[plane], y_maps_[plane]);
    }
    build_sample_map(projection_uv, src_w / 2, src_h / 2, kHalfW, kHalfH,
                     src_w / 2, 2, 1, 0, 0, uv_map_);

    map_src_w_ = src_w;
    map_src_h_ = src_h;
    map_valid_ = true;

    std::fprintf(stderr,
                 "input warp=on frame=%s source=%dx%d intrinsics=(fx=%.2f fy=%.2f cx=%.2f cy=%.2f) "
                 "rpy_deg=(%.3f %.3f %.3f)\n",
                 model_frame_ == ModelFrame::SmallBigModel ? "sbigmodel" : "medmodel",
                 src_w, src_h, fx_, fy_, cx_, cy_,
                 rad_to_deg(roll_), rad_to_deg(pitch_), rad_to_deg(yaw_));
}

bool ModelInputTransform::rvv_available()
{
#if defined(__riscv_vector)
    return true;
#else
    return false;
#endif
}

template <typename OutT>
void ModelInputTransform::warp_scalar(const uint8_t *nv12, int src_w, int src_h,
                                      OutT *out) const
{
    const uint8_t *y_src = nv12;
    const uint8_t *uv_src = nv12 + src_w * src_h;
    const int plane_size = kHalfW * kHalfH;

    for (int plane = 0; plane < 4; ++plane) {
        OutT *dst = out + plane * plane_size;
        const SampleMap &map = y_maps_[plane];
        for (size_t i = 0; i < map.size(); ++i)
            dst[i] = static_cast<OutT>(sample(y_src, map, i, 0));
    }

    OutT *u_plane = out + 4 * plane_size;
    OutT *v_plane = out + 5 * plane_size;
    for (size_t i = 0; i < uv_map_.size(); ++i) {
        u_plane[i] = static_cast<OutT>(sample(uv_src, uv_map_, i, 0));
        v_plane[i] = static_cast<OutT>(sample(uv_src, uv_map_, i, 1));
    }
}

template void ModelInputTransform::warp_scalar<float>(const uint8_t *, int, int,
                                                      float *) const;
template void ModelInputTransform::warp_scalar<uint8_t>(const uint8_t *, int, int,
                                                        uint8_t *) const;

#if defined(__riscv_vector)
namespace {

/* 보간 합(0..255로 클램프된 u32)을 출력 타입에 맞게 저장한다. */
inline void warp_store(float *dst, size_t index, vuint32m4_t sum, size_t vl)
{
    __riscv_vse32_v_f32m4(dst + index, __riscv_vfcvt_f_xu_v_f32m4(sum, vl), vl);
}

inline void warp_store(uint8_t *dst, size_t index, vuint32m4_t sum, size_t vl)
{
    const vuint16m2_t narrow16 = __riscv_vncvt_x_x_w_u16m2(sum, vl);
    __riscv_vse8_v_u8m1(dst + index, __riscv_vncvt_x_x_w_u8m1(narrow16, vl), vl);
}

} // namespace
#endif

template <typename OutT>
void ModelInputTransform::warp_rvv(const uint8_t *nv12, int src_w, int src_h,
                                   OutT *out) const
{
#if defined(__riscv_vector)
    const uint8_t *y_src = nv12;
    const uint8_t *uv_src = nv12 + src_w * src_h;
    const size_t plane_size = static_cast<size_t>(kHalfW) * kHalfH;

    auto sample_plane = [](const uint8_t *src, const SampleMap &map,
                           int channel, OutT *dst) {
        size_t offset_index = 0;
        while (offset_index < map.size()) {
            const size_t vl = __riscv_vsetvl_e32m4(map.size() - offset_index);
            vuint32m4_t offset = __riscv_vle32_v_u32m4(
                map.offset.data() + offset_index, vl);
            if (channel != 0)
                offset = __riscv_vadd_vx_u32m4(offset, static_cast<uint32_t>(channel), vl);
            const vuint16m2_t x_step16 = __riscv_vle16_v_u16m2(
                map.x_step.data() + offset_index, vl);
            const vuint16m2_t y_step16 = __riscv_vle16_v_u16m2(
                map.y_step.data() + offset_index, vl);
            const vuint32m4_t x_step = __riscv_vzext_vf2_u32m4(x_step16, vl);
            const vuint32m4_t y_step = __riscv_vzext_vf2_u32m4(y_step16, vl);
            const vuint32m4_t offset_x = __riscv_vadd_vv_u32m4(offset, x_step, vl);
            const vuint32m4_t offset_y = __riscv_vadd_vv_u32m4(offset, y_step, vl);
            const vuint32m4_t offset_xy = __riscv_vadd_vv_u32m4(offset_y, x_step, vl);

            const vuint8m1_t pixel0 = __riscv_vluxei32_v_u8m1(src, offset, vl);
            const vuint8m1_t pixel1 = __riscv_vluxei32_v_u8m1(src, offset_x, vl);
            const vuint8m1_t pixel2 = __riscv_vluxei32_v_u8m1(src, offset_y, vl);
            const vuint8m1_t pixel3 = __riscv_vluxei32_v_u8m1(src, offset_xy, vl);
            const vuint16m2_t weight0 = __riscv_vle16_v_u16m2(
                map.weight[0].data() + offset_index, vl);
            const vuint16m2_t weight1 = __riscv_vle16_v_u16m2(
                map.weight[1].data() + offset_index, vl);
            const vuint16m2_t weight2 = __riscv_vle16_v_u16m2(
                map.weight[2].data() + offset_index, vl);
            const vuint16m2_t weight3 = __riscv_vle16_v_u16m2(
                map.weight[3].data() + offset_index, vl);

            vuint32m4_t sum = __riscv_vwmulu_vv_u32m4(
                __riscv_vzext_vf2_u16m2(pixel0, vl), weight0, vl);
            sum = __riscv_vwmaccu_vv_u32m4(
                sum, __riscv_vzext_vf2_u16m2(pixel1, vl), weight1, vl);
            sum = __riscv_vwmaccu_vv_u32m4(
                sum, __riscv_vzext_vf2_u16m2(pixel2, vl), weight2, vl);
            sum = __riscv_vwmaccu_vv_u32m4(
                sum, __riscv_vzext_vf2_u16m2(pixel3, vl), weight3, vl);
            sum = __riscv_vadd_vx_u32m4(sum, kWeightScale / 2, vl);
            sum = __riscv_vsrl_vx_u32m4(sum, kWeightBits, vl);
            sum = __riscv_vminu_vx_u32m4(sum, 255, vl);
            warp_store(dst, offset_index, sum, vl);
            offset_index += vl;
        }
    };

    for (int plane = 0; plane < 4; ++plane)
        sample_plane(y_src, y_maps_[plane], 0, out + plane * plane_size);

    // U와 V는 같은 위치와 가중치를 사용하므로 LUT와 주소 계산을 한 번만 한다.
    OutT *u_dst = out + 4 * plane_size;
    OutT *v_dst = out + 5 * plane_size;
    size_t offset_index = 0;
    while (offset_index < uv_map_.size()) {
        const size_t vl = __riscv_vsetvl_e32m4(uv_map_.size() - offset_index);
        const vuint32m4_t offset = __riscv_vle32_v_u32m4(
            uv_map_.offset.data() + offset_index, vl);
        const vuint16m2_t x_step16 = __riscv_vle16_v_u16m2(
            uv_map_.x_step.data() + offset_index, vl);
        const vuint16m2_t y_step16 = __riscv_vle16_v_u16m2(
            uv_map_.y_step.data() + offset_index, vl);
        const vuint32m4_t x_step = __riscv_vzext_vf2_u32m4(x_step16, vl);
        const vuint32m4_t y_step = __riscv_vzext_vf2_u32m4(y_step16, vl);
        const vuint32m4_t offset_x = __riscv_vadd_vv_u32m4(offset, x_step, vl);
        const vuint32m4_t offset_y = __riscv_vadd_vv_u32m4(offset, y_step, vl);
        const vuint32m4_t offset_xy = __riscv_vadd_vv_u32m4(offset_y, x_step, vl);
        const vuint32m4_t offset_v = __riscv_vadd_vx_u32m4(offset, 1, vl);
        const vuint32m4_t offset_vx = __riscv_vadd_vx_u32m4(offset_x, 1, vl);
        const vuint32m4_t offset_vy = __riscv_vadd_vx_u32m4(offset_y, 1, vl);
        const vuint32m4_t offset_vxy = __riscv_vadd_vx_u32m4(offset_xy, 1, vl);

        const vuint16m2_t weight0 = __riscv_vle16_v_u16m2(
            uv_map_.weight[0].data() + offset_index, vl);
        const vuint16m2_t weight1 = __riscv_vle16_v_u16m2(
            uv_map_.weight[1].data() + offset_index, vl);
        const vuint16m2_t weight2 = __riscv_vle16_v_u16m2(
            uv_map_.weight[2].data() + offset_index, vl);
        const vuint16m2_t weight3 = __riscv_vle16_v_u16m2(
            uv_map_.weight[3].data() + offset_index, vl);

        auto interpolate = [&](vuint32m4_t channel_offset,
                               vuint32m4_t channel_offset_x,
                               vuint32m4_t channel_offset_y,
                               vuint32m4_t channel_offset_xy) {
            const vuint8m1_t pixel0 = __riscv_vluxei32_v_u8m1(
                uv_src, channel_offset, vl);
            const vuint8m1_t pixel1 = __riscv_vluxei32_v_u8m1(
                uv_src, channel_offset_x, vl);
            const vuint8m1_t pixel2 = __riscv_vluxei32_v_u8m1(
                uv_src, channel_offset_y, vl);
            const vuint8m1_t pixel3 = __riscv_vluxei32_v_u8m1(
                uv_src, channel_offset_xy, vl);
            vuint32m4_t sum = __riscv_vwmulu_vv_u32m4(
                __riscv_vzext_vf2_u16m2(pixel0, vl), weight0, vl);
            sum = __riscv_vwmaccu_vv_u32m4(
                sum, __riscv_vzext_vf2_u16m2(pixel1, vl), weight1, vl);
            sum = __riscv_vwmaccu_vv_u32m4(
                sum, __riscv_vzext_vf2_u16m2(pixel2, vl), weight2, vl);
            sum = __riscv_vwmaccu_vv_u32m4(
                sum, __riscv_vzext_vf2_u16m2(pixel3, vl), weight3, vl);
            sum = __riscv_vadd_vx_u32m4(sum, kWeightScale / 2, vl);
            sum = __riscv_vsrl_vx_u32m4(sum, kWeightBits, vl);
            return __riscv_vminu_vx_u32m4(sum, 255, vl);
        };
        auto store = [&](OutT *dst, vuint32m4_t sum) {
            warp_store(dst, offset_index, sum, vl);
        };

        store(u_dst, interpolate(offset, offset_x, offset_y, offset_xy));
        store(v_dst, interpolate(offset_v, offset_vx, offset_vy, offset_vxy));
        offset_index += vl;
    }
#else
    warp_scalar(nv12, src_w, src_h, out);
#endif
}

template void ModelInputTransform::warp_rvv<float>(const uint8_t *, int, int,
                                                   float *) const;
template void ModelInputTransform::warp_rvv<uint8_t>(const uint8_t *, int, int,
                                                     uint8_t *) const;

void ModelInputTransform::nv12_to_yuv6_warped_scalar(const uint8_t *nv12,
                                                     int src_w, int src_h,
                                                     float *out)
{
    if (!map_valid_ || src_w != map_src_w_ || src_h != map_src_h_)
        rebuild_maps(src_w, src_h);
    warp_scalar(nv12, src_w, src_h, out);
}

void ModelInputTransform::nv12_to_yuv6_warped_rvv(const uint8_t *nv12,
                                                  int src_w, int src_h,
                                                  float *out)
{
    if (!map_valid_ || src_w != map_src_w_ || src_h != map_src_h_)
        rebuild_maps(src_w, src_h);
    warp_rvv(nv12, src_w, src_h, out);
}

void ModelInputTransform::nv12_to_yuv6_warped(const uint8_t *nv12,
                                              int src_w, int src_h,
                                              float *out)
{
    if (rvv_available() && !scalar_warp_forced())
        nv12_to_yuv6_warped_rvv(nv12, src_w, src_h, out);
    else
        nv12_to_yuv6_warped_scalar(nv12, src_w, src_h, out);
}

void ModelInputTransform::nv12_to_yuv6_warped(const uint8_t *nv12,
                                              int src_w, int src_h,
                                              std::vector<float> &out)
{
    if (out.size() < static_cast<size_t>(6 * kHalfW * kHalfH))
        out.resize(6 * kHalfW * kHalfH);
    nv12_to_yuv6_warped(nv12, src_w, src_h, out.data());
}

void ModelInputTransform::nv12_to_yuv6_warped_scalar(const uint8_t *nv12,
                                                     int src_w, int src_h,
                                                     uint8_t *out)
{
    if (!map_valid_ || src_w != map_src_w_ || src_h != map_src_h_)
        rebuild_maps(src_w, src_h);
    warp_scalar(nv12, src_w, src_h, out);
}

void ModelInputTransform::nv12_to_yuv6_warped_rvv(const uint8_t *nv12,
                                                  int src_w, int src_h,
                                                  uint8_t *out)
{
    if (!map_valid_ || src_w != map_src_w_ || src_h != map_src_h_)
        rebuild_maps(src_w, src_h);
    warp_rvv(nv12, src_w, src_h, out);
}

void ModelInputTransform::nv12_to_yuv6_warped(const uint8_t *nv12,
                                              int src_w, int src_h,
                                              uint8_t *out)
{
    if (rvv_available() && !scalar_warp_forced())
        nv12_to_yuv6_warped_rvv(nv12, src_w, src_h, out);
    else
        nv12_to_yuv6_warped_scalar(nv12, src_w, src_h, out);
}
