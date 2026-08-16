#ifndef SUPERCOMBO_MODEL_H
#define SUPERCOMBO_MODEL_H

#include "ai_base.h"
#include "app_config.h"
#include "model_input_transform.h"

#include <cstddef>
#include <cstdint>
#include <vector>

class SupercomboModel : public AIBase
{
public:
    SupercomboModel(const char *kmodel_file, int debug_mode, const AppConfig &config);

    bool run_frame_nv12(const uint8_t *nv12, int src_w, int src_h, std::vector<float> &raw_output);
    bool run_frame_nv12_stable(const uint8_t *nv12, int src_w, int src_h,
                               std::vector<float> &raw_output);
    void set_input_calibration(const float rpy[3]);
    void set_desire(int desire);

private:
    static constexpr int kModelW = 512;
    static constexpr int kModelH = 256;
    static constexpr int kHalfW = kModelW / 2;
    static constexpr int kHalfH = kModelH / 2;
    static constexpr int kYuv6Floats = 6 * kHalfW * kHalfH;
    static constexpr int kInputImageFloats = 12 * kHalfW * kHalfH;
    static constexpr int kRecurrentFloats = 512;

    bool prepare_image_input(size_t index, ModelInputTransform &transform,
                             const uint8_t *nv12, int src_w, int src_h);
    bool run_frame_nv12_impl(const uint8_t *nv12, int src_w, int src_h,
                             std::vector<float> &raw_output, bool copy_input);
    bool advance_image_history(size_t index);
    bool clear_image_input(size_t index);
    bool write_input(size_t index, const float *data, size_t count);
    size_t shape_count(size_t index) const;
    // image 입력 텐서의 원소 크기. float32 kmodel은 4, uint8 kmodel은 1이다.
    size_t image_elem_bytes(size_t index) const;

    std::vector<runtime_tensor> input_tensors_;
    ModelInputTransform input_transform_;
    ModelInputTransform big_input_transform_;
    std::vector<uint8_t> nv12_cache_;
    std::vector<float> desire_;
    std::vector<float> prev_desire_;
    std::vector<float> traffic_convention_;
    std::vector<float> recurrent_state_;
};

#endif
