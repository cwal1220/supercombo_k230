#ifndef MODEL_INPUT_TRANSFORM_H
#define MODEL_INPUT_TRANSFORM_H

#include "app_config.h"

#include <array>
#include <cstdint>
#include <vector>

enum class ModelFrame {
    MedModel,
    SmallBigModel,
};

class ModelInputTransform
{
public:
    explicit ModelInputTransform(const AppConfig &config,
                                 ModelFrame model_frame = ModelFrame::MedModel);

    void set_calibration(float roll, float pitch, float yaw);
    void projection_matrix(float *projection) const;
    void nv12_to_yuv6_warped(const uint8_t *nv12, int src_w, int src_h, std::vector<float> &out);
    void nv12_to_yuv6_warped(const uint8_t *nv12, int src_w, int src_h, float *out);
    void nv12_to_yuv6_warped(const uint8_t *nv12, int src_w, int src_h, std::vector<uint8_t> &out);
    void nv12_to_yuv6_warped(const uint8_t *nv12, int src_w, int src_h, uint8_t *out);
    void nv12_to_yuv6_warped_scalar(const uint8_t *nv12, int src_w, int src_h, float *out);
    void nv12_to_yuv6_warped_rvv(const uint8_t *nv12, int src_w, int src_h, float *out);
    static bool rvv_available();

private:
    struct SampleMap {
        std::vector<uint32_t> offset;
        std::vector<uint16_t> x_step;
        std::vector<uint16_t> y_step;
        std::array<std::vector<uint16_t>, 4> weight;

        void resize(size_t size);
        size_t size() const { return offset.size(); }
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
                          int bytes_per_pixel, int dst_scale,
                          int dst_x_offset, int dst_y_offset, SampleMap &map) const;
    static uint8_t sample(const uint8_t *base, const SampleMap &map,
                          size_t index, int channel);
    void warp_scalar(const uint8_t *nv12, int src_w, int src_h, float *out) const;
    void warp_rvv(const uint8_t *nv12, int src_w, int src_h, float *out) const;
    void warp_scalar_u8(const uint8_t *nv12, int src_w, int src_h, uint8_t *out) const;
    void warp_rvv_u8(const uint8_t *nv12, int src_w, int src_h, uint8_t *out) const;

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
    ModelFrame model_frame_ = ModelFrame::MedModel;
    std::array<SampleMap, 4> y_maps_;
    SampleMap uv_map_;
};

#endif
