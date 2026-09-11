#ifndef GPU_WARP_H
#define GPU_WARP_H

#include "app_config.h"
#include "vg_lite.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

/* 모델 입력 워프를 VGLite(2.5D GPU)로 수행한다. 퍼스펙티브 3x3과 바이리니어를
 * 하드웨어가 처리하므로 CPU는 소스 평면을 채우는 일만 한다. 출력은 물리 연속
 * 버퍼(nncase pool_shared 텐서)에 직접 쓰므로 복사가 없다. */
class GpuWarp
{
public:
    static constexpr int kTowers = 2;   // medmodel, sbigmodel
    static constexpr int kPlanes = 6;   // Y 4분할 + U + V
    static constexpr int kPlaneW = kDefaultModelWidth / 2;
    static constexpr int kPlaneH = kDefaultModelHeight / 2;

    /* CPU가 채워야 하는 소스. GPU는 L8 평면만 읽으므로 UV는 이후 분리한다.
     * chroma는 일반 메모리 스테이징이다. 공유메모리에서 벡터 로드를 하면
     * C908에서 불안정하므로 링 복사는 여기로 받는다. */
    struct Planes {
        uint8_t *luma = nullptr;
        size_t luma_stride = 0;
        uint8_t *chroma = nullptr;
        size_t chroma_stride = 0;
    };

    static std::unique_ptr<GpuWarp> create(int src_w, int src_h);
    ~GpuWarp();

    GpuWarp(const GpuWarp &) = delete;
    GpuWarp &operator=(const GpuWarp &) = delete;

    bool accepts(int src_w, int src_h) const { return src_w == src_w_ && src_h == src_h_; }
    bool bind_output(int tower, uint8_t *cpu, uintptr_t physical);
    /* projection은 ModelInputTransform과 같은 모델 좌표 -> 소스 픽셀 3x3. */
    void set_projection(int tower, const float *projection);
    Planes planes();
    void upload(const uint8_t *nv12);
    /* Planes.chroma에 받아둔 UV를 U/V 평면으로 나눈다. */
    void split_chroma();
    bool run();

private:
    GpuWarp() = default;

    int src_w_ = 0;
    int src_h_ = 0;
    bool initialized_ = false;
    std::vector<uint8_t> chroma_staging_;
    vg_lite_buffer_t luma_{};
    vg_lite_buffer_t u_{};
    vg_lite_buffer_t v_{};
    vg_lite_buffer_t target_[kTowers * kPlanes]{};
    vg_lite_matrix_t matrix_[kTowers * kPlanes]{};
    bool bound_[kTowers] = {false, false};
};

#endif
