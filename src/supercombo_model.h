#ifndef SUPERCOMBO_MODEL_H
#define SUPERCOMBO_MODEL_H

#include "ai_base.h"

#include <cstddef>
#include <cstdint>
#include <vector>

class SupercomboModel : public AIBase
{
public:
    SupercomboModel(const char *kmodel_file, int debug_mode);

    bool run_frame_nv12(const uint8_t *nv12, int src_w, int src_h, std::vector<float> &raw_output);
    void reset_state();

private:
    static constexpr int kModelW = 512;
    static constexpr int kModelH = 256;
    static constexpr int kHalfW = kModelW / 2;
    static constexpr int kHalfH = kModelH / 2;
    static constexpr int kYuv6Floats = 6 * kHalfW * kHalfH;
    static constexpr int kInputImageFloats = 12 * kHalfW * kHalfH;
    static constexpr int kRecurrentFloats = 512;

    void nv12_to_yuv6(const uint8_t *nv12, int src_w, int src_h, std::vector<float> &out);
    bool run_current_yuv6(std::vector<float> &raw_output, bool profile, uint64_t t0, uint64_t t1);
    bool write_input(size_t index, const float *data, size_t count);
    size_t shape_count(size_t index) const;

    std::vector<runtime_tensor> input_tensors_;
    std::vector<float> prev_yuv_;
    std::vector<float> current_yuv_;
    std::vector<float> input_imgs_;
    std::vector<float> big_input_imgs_;
    std::vector<float> desire_;
    std::vector<float> traffic_convention_;
    std::vector<float> recurrent_state_;
};

#endif
