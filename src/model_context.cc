#include "model_context.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

uint16_t model_float_to_half(float value)
{
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    const uint16_t sign = static_cast<uint16_t>((bits >> 16) & 0x8000U);
    const uint32_t exponent = (bits >> 23) & 0xffU;
    uint32_t mantissa = bits & 0x7fffffU;

    if (exponent == 0xffU) {
        if (mantissa == 0) return static_cast<uint16_t>(sign | 0x7c00U);
        return static_cast<uint16_t>(sign | 0x7e00U);
    }

    int half_exponent = static_cast<int>(exponent) - 127 + 15;
    if (half_exponent >= 31)
        return static_cast<uint16_t>(sign | 0x7c00U);

    if (half_exponent <= 0) {
        if (half_exponent < -10) return sign;
        mantissa |= 0x800000U;
        const int shift = 14 - half_exponent;
        uint32_t half_mantissa = mantissa >> shift;
        const uint32_t remainder = mantissa & ((1U << shift) - 1U);
        const uint32_t halfway = 1U << (shift - 1);
        if (remainder > halfway || (remainder == halfway && (half_mantissa & 1U)))
            ++half_mantissa;
        return static_cast<uint16_t>(sign | half_mantissa);
    }

    uint32_t half_mantissa = mantissa >> 13;
    const uint32_t remainder = mantissa & 0x1fffU;
    if (remainder > 0x1000U || (remainder == 0x1000U && (half_mantissa & 1U))) {
        ++half_mantissa;
        if (half_mantissa == 0x400U) {
            half_mantissa = 0;
            ++half_exponent;
            if (half_exponent >= 31)
                return static_cast<uint16_t>(sign | 0x7c00U);
        }
    }
    return static_cast<uint16_t>(sign |
        (static_cast<uint16_t>(half_exponent) << 10) |
        static_cast<uint16_t>(half_mantissa));
}

void model_encode_float16(const float *src, size_t count, uint16_t *dst)
{
    if ((!src || !dst) && count != 0)
        throw std::invalid_argument("null float16 conversion buffer");
    for (size_t i = 0; i < count; ++i)
        dst[i] = model_float_to_half(src[i]);
}

void ModernModelContext::reset()
{
    feature_history_.fill(0.0f);
    desire_history_.fill(0.0f);
    previous_hidden_.fill(0.0f);
    feature_head_ = 0;
    desire_head_ = 0;
}

void ModernModelContext::prepare_feature_input(uint16_t *dst, size_t count)
{
    if (!dst || count != kFeatureSamples * kFeatureCount)
        throw std::invalid_argument("bad features_buffer destination");

    float *newest = feature_history_.data() + feature_head_ * kFeatureCount;
    std::copy(previous_hidden_.begin(), previous_hidden_.end(), newest);
    feature_head_ = (feature_head_ + 1) % kFeatureHistoryFrames;

    for (size_t sample = 0; sample < kFeatureSamples; ++sample) {
        const size_t logical_frame = sample * kFrameSkip;
        const size_t physical_frame = (feature_head_ + logical_frame) % kFeatureHistoryFrames;
        model_encode_float16(feature_history_.data() + physical_frame * kFeatureCount,
                             kFeatureCount, dst + sample * kFeatureCount);
    }
}

void ModernModelContext::prepare_desire_input(const float *pulse, size_t pulse_count,
                                               uint16_t *dst, size_t count)
{
    if (!pulse || pulse_count != kDesireCount || !dst ||
        count != kDesireSamples * kDesireCount) {
        throw std::invalid_argument("bad desire_pulse buffers");
    }

    float *newest = desire_history_.data() + desire_head_ * kDesireCount;
    std::copy(pulse, pulse + kDesireCount, newest);
    desire_head_ = (desire_head_ + 1) % kDesireHistoryFrames;

    for (size_t sample = 0; sample < kDesireSamples; ++sample) {
        for (size_t desire = 0; desire < kDesireCount; ++desire) {
            float maximum = 0.0f;
            for (size_t frame = 0; frame < kFrameSkip; ++frame) {
                const size_t logical_frame = sample * kFrameSkip + frame;
                const size_t physical_frame = (desire_head_ + logical_frame) % kDesireHistoryFrames;
                maximum = std::max(maximum,
                    desire_history_[physical_frame * kDesireCount + desire]);
            }
            dst[sample * kDesireCount + desire] = model_float_to_half(maximum);
        }
    }
}

bool ModernModelContext::set_previous_hidden(const float *hidden, size_t count)
{
    if (!hidden || count != kFeatureCount) return false;
    for (size_t i = 0; i < count; ++i) {
        if (!std::isfinite(hidden[i])) return false;
    }
    std::copy(hidden, hidden + count, previous_hidden_.begin());
    return true;
}
