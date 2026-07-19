#ifndef MODEL_INPUT_TRANSFORM_H
#define MODEL_INPUT_TRANSFORM_H

#include "app_config.h"

#include <cstdint>
#include <vector>

class ModelInputTransform
{
public:
    explicit ModelInputTransform(const AppConfig &config);

    void set_calibration(float roll, float pitch, float yaw);
    void projection_matrix(float *projection) const;
    void nv12_to_yuv6_warped(const uint8_t *nv12, int src_w, int src_h, std::vector<float> &out);

private:
    struct BilinearSample {
        uint32_t offset[4] = {0, 0, 0, 0};
        uint16_t weight[4] = {0, 0, 0, 0};
    };

    static constexpr int kModelW = kDefaultModelWidth;
    static constexpr int kModelH = kDefaultModelHeight;
    static constexpr int kHalfW = kModelW / 2;
    static constexpr int kHalfH = kModelH / 2;
    static constexpr int kWeightBits = 12;
    static constexpr int kWeightScale = 1 << kWeightBits;

    void rebuild_maps(int src_w, int src_h);
    void build_sample_map(const float *projection, int src_w, int src_h,
                          int dst_w, int dst_h, int src_stride_pixels,
                          int bytes_per_pixel, std::vector<BilinearSample> &map) const;
    static uint8_t sample_luma(const uint8_t *base, const BilinearSample &sample);
    static uint8_t sample_chroma(const uint8_t *base, const BilinearSample &sample, int channel);

    bool map_valid_ = false;
    int map_src_w_ = 0;
    int map_src_h_ = 0;

    float fx_ = kDefaultInputWarpFx;
    float fy_ = kDefaultInputWarpFy;
    float cx_ = kDefaultInputWarpCx;
    float cy_ = kDefaultInputWarpCy;
    float height_ = 1.22f;
    float roll_ = 0.0f;
    float pitch_ = 0.0f;
    float yaw_ = 0.0f;
    std::vector<BilinearSample> y_map_;
    std::vector<BilinearSample> uv_map_;
};

#endif
