#ifndef MODEL_INPUT_TRANSFORM_H
#define MODEL_INPUT_TRANSFORM_H

#include "app_config.h"

#include <cstdint>
#include <vector>

class ModelInputTransform
{
public:
    explicit ModelInputTransform(const AppConfig &config);

    void nv12_to_yuv6_warped(const uint8_t *nv12, int src_w, int src_h, std::vector<float> &out);

private:
    struct BilinearSample {
        uint32_t offset[4] = {0, 0, 0, 0};
        uint16_t weight[4] = {0, 0, 0, 0};
    };

    static constexpr int kModelW = 512;
    static constexpr int kModelH = 256;
    static constexpr int kHalfW = kModelW / 2;
    static constexpr int kHalfH = kModelH / 2;
    static constexpr int kWeightBits = 12;
    static constexpr int kWeightScale = 1 << kWeightBits;

    void rebuild_maps(int src_w, int src_h);
    void build_projection(float *projection) const;
    void build_sample_map(const float *projection, int src_w, int src_h,
                          int dst_w, int dst_h, int src_stride_pixels,
                          int bytes_per_pixel, std::vector<BilinearSample> &map) const;
    static uint8_t sample_luma(const uint8_t *base, const BilinearSample &sample);
    static uint8_t sample_chroma(const uint8_t *base, const BilinearSample &sample, int channel);

    bool map_valid_ = false;
    int map_src_w_ = 0;
    int map_src_h_ = 0;

    float fx_ = 910.0f;
    float fy_ = 910.0f;
    float cx_ = 256.0f;
    float cy_ = 47.6f;
    float height_ = 1.22f;
    float roll_ = 0.0f;
    float pitch_ = 0.0f;
    float yaw_ = 0.0f;

    std::vector<BilinearSample> y_map_;
    std::vector<BilinearSample> uv_map_;
};

#endif
