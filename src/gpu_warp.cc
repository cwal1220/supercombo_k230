#include "gpu_warp.h"

#include "model_input_transform.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace {

/* NV12의 인터리브 UV를 U/V 평면으로 나눈다. GPU는 L8 평면만 읽는다. */
void uv_split(const uint8_t *uv, int width, int height, uint8_t *u, uint8_t *v, size_t stride)
{
    for (int row = 0; row < height; ++row) {
        const uint8_t *source = uv + static_cast<size_t>(row) * width * 2;
        uint8_t *u_row = u + static_cast<size_t>(row) * stride;
        uint8_t *v_row = v + static_cast<size_t>(row) * stride;
#if defined(__riscv_vector)
        size_t remaining = static_cast<size_t>(width);
        while (remaining) {
            const size_t vl = __riscv_vsetvl_e8m4(remaining);
            const vuint8m4x2_t pair = __riscv_vlseg2e8_v_u8m4x2(source, vl);
            __riscv_vse8_v_u8m4(u_row, __riscv_vget_v_u8m4x2_u8m4(pair, 0), vl);
            __riscv_vse8_v_u8m4(v_row, __riscv_vget_v_u8m4x2_u8m4(pair, 1), vl);
            source += 2 * vl;
            u_row += vl;
            v_row += vl;
            remaining -= vl;
        }
#else
        for (int column = 0; column < width; ++column) {
            u_row[column] = source[2 * column];
            v_row[column] = source[2 * column + 1];
        }
#endif
    }
}

/* VGLite 블릿 행렬은 소스 -> 목적지라 앱의 목적지 -> 소스 행렬을 뒤집어 준다. */
bool invert3(const float *m, float *out)
{
    const float a = m[0], b = m[1], c = m[2];
    const float d = m[3], e = m[4], f = m[5];
    const float g = m[6], h = m[7], i = m[8];
    const float determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    if (std::fabs(determinant) < 1e-12f)
        return false;
    const float scale = 1.0f / determinant;
    out[0] = (e * i - f * h) * scale;
    out[1] = (c * h - b * i) * scale;
    out[2] = (b * f - c * e) * scale;
    out[3] = (f * g - d * i) * scale;
    out[4] = (a * i - c * g) * scale;
    out[5] = (c * d - a * f) * scale;
    out[6] = (d * h - e * g) * scale;
    out[7] = (b * g - a * h) * scale;
    out[8] = (a * e - b * d) * scale;
    return true;
}

void matmul3(const float *a, const float *b, float *out)
{
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            float sum = 0.0f;
            for (int k = 0; k < 3; ++k)
                sum += a[row * 3 + k] * b[k * 3 + column];
            out[row * 3 + column] = sum;
        }
    }
}

/* 출력 픽셀 -> 모델 좌표 변환을 접어 넣는다. VGLite는 바이리니어에서 픽셀
 * 중심을 반 픽셀 당기므로 -0.5로 상쇄해야 CPU 워프와 같은 표본이 나온다. */
void plane_matrix(const float *projection, int scale, int x_offset, int y_offset,
                  vg_lite_matrix_t *matrix)
{
    const float step[9] = {
        static_cast<float>(scale), 0.0f, static_cast<float>(x_offset) - 0.5f,
        0.0f, static_cast<float>(scale), static_cast<float>(y_offset) - 0.5f,
        0.0f, 0.0f, 1.0f,
    };
    float combined[9];
    float inverse[9];
    matmul3(projection, step, combined);
    if (!invert3(combined, inverse))
        return;
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
            matrix->m[row][column] = inverse[row * 3 + column];
}

bool allocate_plane(vg_lite_buffer_t &buffer, int width, int height)
{
    buffer.width = width;
    buffer.height = height;
    buffer.format = VG_LITE_L8;
    return vg_lite_allocate(&buffer) == VG_LITE_SUCCESS;
}

} // namespace

std::unique_ptr<GpuWarp> GpuWarp::create(int src_w, int src_h)
{
    if (src_w <= 0 || src_h <= 0 || (src_w & 1) || (src_h & 1))
        return nullptr;

    std::unique_ptr<GpuWarp> warp(new GpuWarp());
    warp->src_w_ = src_w;
    warp->src_h_ = src_h;
    if (vg_lite_init(0, 0) != VG_LITE_SUCCESS) {
        std::fprintf(stderr, "gpu warp: vg_lite_init failed, falling back to CPU\n");
        return nullptr;
    }
    warp->initialized_ = true;

    warp->chroma_staging_.resize(static_cast<size_t>(src_w) * (src_h / 2));
    if (!allocate_plane(warp->luma_, src_w, src_h) ||
        !allocate_plane(warp->u_, src_w / 2, src_h / 2) ||
        !allocate_plane(warp->v_, src_w / 2, src_h / 2)) {
        std::fprintf(stderr, "gpu warp: source allocation failed, falling back to CPU\n");
        return nullptr;
    }
    return warp;
}

GpuWarp::~GpuWarp()
{
    if (!initialized_)
        return;
    for (vg_lite_buffer_t *buffer : {&luma_, &u_, &v_})
        if (buffer->handle)
            vg_lite_free(buffer);
    vg_lite_close();
}

bool GpuWarp::bind_output(int tower, uint8_t *cpu, uintptr_t physical)
{
    if (tower < 0 || tower >= kTowers || !cpu || physical == 0)
        return false;

    constexpr size_t plane_bytes = static_cast<size_t>(kPlaneW) * kPlaneH;
    for (int plane = 0; plane < kPlanes; ++plane) {
        vg_lite_buffer_t &buffer = target_[tower * kPlanes + plane];
        buffer = vg_lite_buffer_t{};
        buffer.width = kPlaneW;
        buffer.height = kPlaneH;
        buffer.stride = kPlaneW;
        buffer.format = VG_LITE_L8;
        buffer.tiled = VG_LITE_LINEAR;
        buffer.memory = cpu + plane * plane_bytes;
        buffer.address = static_cast<vg_lite_uint32_t>(physical + plane * plane_bytes);
    }
    bound_[tower] = true;
    return true;
}

void GpuWarp::set_projection(int tower, const float *projection)
{
    if (tower < 0 || tower >= kTowers || !projection)
        return;

    float chroma_projection[9];
    projection_scale_buffer(projection, 0.5f, chroma_projection);

    constexpr int x_offsets[4] = {0, 0, 1, 1};
    constexpr int y_offsets[4] = {0, 1, 0, 1};
    vg_lite_matrix_t *matrix = &matrix_[tower * kPlanes];
    for (int plane = 0; plane < 4; ++plane)
        plane_matrix(projection, 2, x_offsets[plane], y_offsets[plane], &matrix[plane]);
    plane_matrix(chroma_projection, 1, 0, 0, &matrix[4]);
    matrix[5] = matrix[4];
}

GpuWarp::Planes GpuWarp::planes()
{
    Planes planes;
    planes.luma = static_cast<uint8_t *>(luma_.memory);
    planes.luma_stride = static_cast<size_t>(luma_.stride);
    planes.chroma = chroma_staging_.data();
    planes.chroma_stride = static_cast<size_t>(src_w_);
    return planes;
}

void GpuWarp::upload(const uint8_t *nv12)
{
    if (!nv12)
        return;

    uint8_t *luma = static_cast<uint8_t *>(luma_.memory);
    for (int row = 0; row < src_h_; ++row)
        std::memcpy(luma + static_cast<size_t>(row) * luma_.stride,
                    nv12 + static_cast<size_t>(row) * src_w_, src_w_);
    uv_split(nv12 + static_cast<size_t>(src_w_) * src_h_, src_w_ / 2, src_h_ / 2,
                  static_cast<uint8_t *>(u_.memory), static_cast<uint8_t *>(v_.memory),
                  static_cast<size_t>(u_.stride));
}

void GpuWarp::split_chroma()
{
    uv_split(chroma_staging_.data(), src_w_ / 2, src_h_ / 2,
                  static_cast<uint8_t *>(u_.memory), static_cast<uint8_t *>(v_.memory),
                  static_cast<size_t>(u_.stride));
}

bool GpuWarp::run()
{
    for (int tower = 0; tower < kTowers; ++tower) {
        if (!bound_[tower])
            return false;
        vg_lite_buffer_t *target = &target_[tower * kPlanes];
        vg_lite_matrix_t *matrix = &matrix_[tower * kPlanes];
        for (int plane = 0; plane < kPlanes; ++plane) {
            vg_lite_buffer_t *source = plane < 4 ? &luma_ : (plane == 4 ? &u_ : &v_);
            if (vg_lite_blit(&target[plane], source, &matrix[plane], VG_LITE_BLEND_NONE, 0,
                             VG_LITE_FILTER_BI_LINEAR) != VG_LITE_SUCCESS)
                return false;
        }
    }
    return vg_lite_finish() == VG_LITE_SUCCESS;
}
