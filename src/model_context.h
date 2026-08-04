#ifndef MODEL_CONTEXT_H
#define MODEL_CONTEXT_H

#include <array>
#include <cstddef>
#include <cstdint>

uint16_t model_float_to_half(float value);
void model_encode_float16(const float *src, size_t count, uint16_t *dst);

class ModernModelContext
{
public:
    static constexpr size_t kFrameSkip = 4;
    static constexpr size_t kFeatureCount = 512;
    static constexpr size_t kFeatureSamples = 24;
    static constexpr size_t kFeatureHistoryFrames = kFrameSkip * kFeatureSamples;
    static constexpr size_t kDesireCount = 8;
    static constexpr size_t kDesireSamples = 25;
    static constexpr size_t kDesireHistoryFrames = kFrameSkip * kDesireSamples;

    void reset();
    void prepare_feature_input(uint16_t *dst, size_t count);
    void prepare_desire_input(const float *pulse, size_t pulse_count,
                              uint16_t *dst, size_t count);
    bool set_previous_hidden(const float *hidden, size_t count);

private:
    std::array<float, kFeatureHistoryFrames * kFeatureCount> feature_history_{};
    std::array<float, kDesireHistoryFrames * kDesireCount> desire_history_{};
    std::array<float, kFeatureCount> previous_hidden_{};
    size_t feature_head_ = 0;
    size_t desire_head_ = 0;
};

#endif
