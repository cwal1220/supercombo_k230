#include "model_context.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

namespace {

bool all_zero(const uint16_t *values, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        if (values[i] != 0) return false;
    }
    return true;
}

} // namespace

int main()
{
    if (model_float_to_half(0.0f) != 0x0000U ||
        model_float_to_half(1.0f) != 0x3c00U ||
        model_float_to_half(-2.0f) != 0xc000U ||
        model_float_to_half(65504.0f) != 0x7bffU ||
        model_float_to_half(std::numeric_limits<float>::infinity()) != 0x7c00U ||
        model_half_to_float(0x0001U) != std::ldexp(1.0f, -24) ||
        model_half_to_float(0x3c00U) != 1.0f ||
        model_half_to_float(0xc000U) != -2.0f) {
        std::cerr << "float16 encoding failed\n";
        return 1;
    }

    ModernModelContext context;
    std::array<float, ModernModelContext::kFeatureCount> hidden{};
    std::array<uint16_t, ModernModelContext::kFeatureSamples * ModernModelContext::kFeatureCount> features{};
    for (int frame = 1; frame <= 5; ++frame) {
        hidden.fill(static_cast<float>(frame));
        if (!context.set_previous_hidden(hidden.data(), hidden.size())) return 1;
        context.prepare_feature_input(features.data(), features.size());
        const float expected_last = frame < 4 ? 0.0f : static_cast<float>(frame - 3);
        if (model_half_to_float(features[(ModernModelContext::kFeatureSamples - 1) *
                                         ModernModelContext::kFeatureCount]) != expected_last) {
            std::cerr << "feature cadence failed at frame " << frame << "\n";
            return 1;
        }
    }

    context.reset();
    std::array<float, ModernModelContext::kDesireCount> pulse{};
    std::array<uint16_t, ModernModelContext::kDesireSamples * ModernModelContext::kDesireCount> desire{};
    pulse[3] = 1.0f;
    context.prepare_desire_input(pulse.data(), pulse.size(), desire.data(), desire.size());
    const size_t newest = (ModernModelContext::kDesireSamples - 1) * ModernModelContext::kDesireCount + 3;
    if (model_half_to_float(desire[newest]) != 1.0f) {
        std::cerr << "desire pulse was not added to newest bucket\n";
        return 1;
    }
    pulse.fill(0.0f);
    for (int i = 0; i < 4; ++i)
        context.prepare_desire_input(pulse.data(), pulse.size(), desire.data(), desire.size());
    const size_t previous_bucket = (ModernModelContext::kDesireSamples - 2) *
        ModernModelContext::kDesireCount + 3;
    if (model_half_to_float(desire[newest]) != 0.0f ||
        model_half_to_float(desire[previous_bucket]) != 1.0f) {
        std::cerr << "desire temporal pooling cadence failed\n";
        return 1;
    }

    context.reset();
    context.prepare_feature_input(features.data(), features.size());
    if (!all_zero(features.data(), features.size())) {
        std::cerr << "context reset failed\n";
        return 1;
    }
    std::cout << "MODEL_CONTEXT_OK frame_skip=4 features=24x512 desire=25x8\n";
    return 0;
}
