#include "nv12_rgb_converter.h"

#include <algorithm>
#include <stdexcept>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace {

#if defined(__riscv_vector)

// Y 값을 scalar 기준식과 같은 32-bit BT.601 limited-range 값으로 확장한다.
vint32m4_t make_y_c(vuint8m1_t y8, size_t vl)
{
    vuint16m2_t y16u = __riscv_vwcvtu_x_x_v_u16m2(y8, vl);
    vint16m2_t y16 = __riscv_vreinterpret_v_u16m2_i16m2(y16u);
    vint32m4_t c = __riscv_vwsub_vx_i32m4(y16, 16, vl);
    return __riscv_vmax_vx_i32m4(c, 0, vl);
}

// U/V 값을 signed chroma 값으로 확장한다.
vint32m4_t make_chroma(vuint8m1_t chroma8, size_t vl)
{
    vuint16m2_t chroma16u = __riscv_vwcvtu_x_x_v_u16m2(chroma8, vl);
    vint16m2_t chroma16 = __riscv_vreinterpret_v_u16m2_i16m2(chroma16u);
    return __riscv_vwsub_vx_i32m4(chroma16, 128, vl);
}

// 8.8 fixed-point 값을 0..255로 clamp한 뒤 u8로 pack한다.
vuint8m1_t pack_u8(vint32m4_t value, size_t vl)
{
    value = __riscv_vmax_vx_i32m4(value, 0, vl);
    value = __riscv_vmin_vx_i32m4(value, 255 * 256, vl);
    vuint32m4_t unsigned_value = __riscv_vreinterpret_v_i32m4_u32m4(value);
    vuint16m2_t narrowed = __riscv_vnsrl_wx_u16m2(unsigned_value, 8, vl);
    return __riscv_vnsrl_wx_u8m1(narrowed, 0, vl);
}

void convert_rows_rvv(const uint8_t *nv12, uint8_t *rgb, unsigned width,
                      unsigned height, unsigned row_begin, unsigned row_end)
{
    const uint8_t *y_plane = nv12;
    const uint8_t *uv_plane = nv12 + static_cast<size_t>(width) * height;
    for (unsigned y = row_begin; y < row_end; ++y) {
        const uint8_t *y_row = y_plane + static_cast<size_t>(y) * width;
        const uint8_t *uv_row = uv_plane + static_cast<size_t>(y / 2u) * width;
        uint8_t *dst = rgb + static_cast<size_t>(y - row_begin) * width * 3u;
        unsigned pairs_left = width / 2u;
        while (pairs_left > 0) {
            const size_t vl = __riscv_vsetvl_e8m1(pairs_left);
            const vuint8m1_t y0_8 = __riscv_vlse8_v_u8m1(y_row, 2, vl);
            const vuint8m1_t y1_8 = __riscv_vlse8_v_u8m1(y_row + 1, 2, vl);
            const vuint8m1_t u_8 = __riscv_vlse8_v_u8m1(uv_row, 2, vl);
            const vuint8m1_t v_8 = __riscv_vlse8_v_u8m1(uv_row + 1, 2, vl);

            const vint32m4_t u = make_chroma(u_8, vl);
            const vint32m4_t v = make_chroma(v_8, vl);
            const vint32m4_t u_b = __riscv_vmul_vx_i32m4(u, 516, vl);
            const vint32m4_t u_g = __riscv_vmul_vx_i32m4(u, 100, vl);
            const vint32m4_t v_g = __riscv_vmul_vx_i32m4(v, 208, vl);
            const vint32m4_t v_r = __riscv_vmul_vx_i32m4(v, 409, vl);

            vint32m4_t base0 = __riscv_vmul_vx_i32m4(make_y_c(y0_8, vl), 298, vl);
            base0 = __riscv_vadd_vx_i32m4(base0, 128, vl);
            vint32m4_t base1 = __riscv_vmul_vx_i32m4(make_y_c(y1_8, vl), 298, vl);
            base1 = __riscv_vadd_vx_i32m4(base1, 128, vl);

            const vuint8m1_t b0 = pack_u8(__riscv_vadd_vv_i32m4(base0, u_b, vl), vl);
            const vuint8m1_t r0 = pack_u8(__riscv_vadd_vv_i32m4(base0, v_r, vl), vl);
            vint32m4_t g0 = __riscv_vsub_vv_i32m4(base0, u_g, vl);
            g0 = __riscv_vsub_vv_i32m4(g0, v_g, vl);
            const vuint8m1_t g0_8 = pack_u8(g0, vl);

            const vuint8m1_t b1 = pack_u8(__riscv_vadd_vv_i32m4(base1, u_b, vl), vl);
            const vuint8m1_t r1 = pack_u8(__riscv_vadd_vv_i32m4(base1, v_r, vl), vl);
            vint32m4_t g1 = __riscv_vsub_vv_i32m4(base1, u_g, vl);
            g1 = __riscv_vsub_vv_i32m4(g1, v_g, vl);
            const vuint8m1_t g1_8 = pack_u8(g1, vl);

            // RGB 순서로 저장한다 (이식 원본은 BGR).
            __riscv_vsse8_v_u8m1(dst + 0, 6, r0, vl);
            __riscv_vsse8_v_u8m1(dst + 1, 6, g0_8, vl);
            __riscv_vsse8_v_u8m1(dst + 2, 6, b0, vl);
            __riscv_vsse8_v_u8m1(dst + 3, 6, r1, vl);
            __riscv_vsse8_v_u8m1(dst + 4, 6, g1_8, vl);
            __riscv_vsse8_v_u8m1(dst + 5, 6, b1, vl);

            y_row += vl * 2u;
            uv_row += vl * 2u;
            dst += vl * 6u;
            pairs_left -= static_cast<unsigned>(vl);
        }
    }
}

#else

uint8_t clamp_u8(int value)
{
    return static_cast<uint8_t>(std::min(255, std::max(0, value)));
}

void convert_rows_scalar(const uint8_t *nv12, uint8_t *rgb, unsigned width,
                         unsigned height, unsigned row_begin, unsigned row_end)
{
    const uint8_t *y_plane = nv12;
    const uint8_t *uv_plane = nv12 + static_cast<size_t>(width) * height;
    for (unsigned y = row_begin; y < row_end; ++y) {
        const uint8_t *y_row = y_plane + static_cast<size_t>(y) * width;
        const uint8_t *uv_row = uv_plane + static_cast<size_t>(y / 2u) * width;
        uint8_t *dst = rgb + static_cast<size_t>(y - row_begin) * width * 3u;
        for (unsigned x = 0; x < width; ++x) {
            const int c = std::max(0, static_cast<int>(y_row[x]) - 16);
            const int d = static_cast<int>(uv_row[(x / 2u) * 2u]) - 128;
            const int e = static_cast<int>(uv_row[(x / 2u) * 2u + 1u]) - 128;
            const int base = 298 * c + 128;
            dst[x * 3u + 0u] = clamp_u8((base + 409 * e) >> 8);
            dst[x * 3u + 1u] = clamp_u8((base - 100 * d - 208 * e) >> 8);
            dst[x * 3u + 2u] = clamp_u8((base + 516 * d) >> 8);
        }
    }
}

#endif

} // namespace

// 변환할 소스 크기와 행 구간을 설정한다.
void Nv12RgbConverter::open(unsigned width, unsigned height, unsigned row_begin,
                            unsigned row_end)
{
    if ((width % 2u) != 0u || (height % 2u) != 0u ||
        (row_begin % 2u) != 0u || (row_end % 2u) != 0u ||
        row_begin >= row_end || row_end > height) {
        throw std::runtime_error("NV12 RGB converter requires even aligned rows");
    }
    width_ = width;
    height_ = height;
    row_begin_ = row_begin;
    row_end_ = row_end;
    rgb_.assign(static_cast<size_t>(width_) * (row_end_ - row_begin_) * 3u, 0);
}

// nv12에서 설정된 행 구간을 RGB24로 변환해 내부 버퍼에 담는다.
void Nv12RgbConverter::convert(const uint8_t *nv12)
{
    if (width_ == 0 || !nv12) throw std::runtime_error("NV12 RGB converter is not open");
#if defined(__riscv_vector)
    convert_rows_rvv(nv12, rgb_.data(), width_, height_, row_begin_, row_end_);
#else
    convert_rows_scalar(nv12, rgb_.data(), width_, height_, row_begin_, row_end_);
#endif
}
