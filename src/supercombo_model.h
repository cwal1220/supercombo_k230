#ifndef SUPERCOMBO_MODEL_H
#define SUPERCOMBO_MODEL_H

#include "ai_base.h"
#include "app_config.h"
#include "model_context.h"
#include "model_input_transform.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

class SupercomboModel : public AIBase
{
public:
    SupercomboModel(const char *kmodel_file, int debug_mode, const AppConfig &config);

    bool run_frame_nv12(const uint8_t *nv12, int src_w, int src_h, std::vector<float> &raw_output);
    void set_input_calibration(const float rpy[3]);
    void set_desire(int desire);

private:
    static constexpr size_t kModelW = 512;
    static constexpr size_t kModelH = 256;
    static constexpr size_t kHalfW = kModelW / 2;
    static constexpr size_t kHalfH = kModelH / 2;
    static constexpr size_t kYuv6Bytes = 6 * kHalfW * kHalfH;
    static constexpr size_t kInputImageBytes = 2 * kYuv6Bytes;
    static constexpr size_t kImageHistoryFrames = ModernModelContext::kFrameSkip;
    static constexpr size_t kOutputFloats = 2576;
    static constexpr size_t kHiddenOffset = 1064;

    using ImageHistory = std::array<std::array<uint8_t, kYuv6Bytes>, kImageHistoryFrames>;

    void validate_model_abi() const;
    void reset_temporal_state();
    bool prepare_image_input(size_t index, ModelInputTransform &transform,
                             ImageHistory &history, const uint8_t *nv12,
                             int src_w, int src_h);
    bool commit_image_history(size_t index, ImageHistory &history);
    bool write_input_bytes(size_t index, const void *data, size_t bytes);
    bool write_input_f16(size_t index, const uint16_t *data, size_t count);
    size_t shape_count(size_t index) const;

    std::vector<runtime_tensor> input_tensors_;
    ModelInputTransform input_transform_;
    ModelInputTransform big_input_transform_;
    std::vector<uint8_t> nv12_cache_;
    std::array<float, ModernModelContext::kDesireCount> desire_pulse_{};
    std::array<float, ModernModelContext::kDesireCount> previous_desire_{};
    std::array<uint16_t, ModernModelContext::kFeatureSamples * ModernModelContext::kFeatureCount> feature_input_{};
    std::array<uint16_t, ModernModelContext::kDesireSamples * ModernModelContext::kDesireCount> desire_input_{};
    std::array<uint16_t, 2> traffic_input_{};
    std::array<uint16_t, 2> action_input_{};
    ImageHistory image_history_{};
    ImageHistory big_image_history_{};
    size_t image_history_head_ = 0;
    ModernModelContext model_context_;
};

#endif
