#ifndef COMMON_UTILS_H
#define COMMON_UTILS_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <csignal>
#include <sys/time.h>
#include <time.h>

inline float clamp_float(float value, float lo, float hi)
{
    if (!std::isfinite(value)) return lo;
    return std::min(std::max(value, lo), hi);
}

inline int clamp_int(float value, int lo, int hi)
{
    return std::min(std::max(static_cast<int>(std::lround(value)), lo), hi);
}

inline int clamp_int(int value, int lo, int hi)
{
    return std::min(std::max(value, lo), hi);
}

inline uint32_t get_signal_le(const uint8_t *data, int start_bit, int length)
{
    uint32_t raw = 0;
    for (int i = 0; i < length; ++i) {
        const int bit = start_bit + i;
        if (data[bit / 8] & (1U << (bit % 8)))
            raw |= 1U << i;
    }
    return raw;
}

inline uint64_t timeval_us(const timeval &tv)
{
    return static_cast<uint64_t>(tv.tv_sec) * 1000000ULL + tv.tv_usec;
}

inline uint64_t k230_now_ns()
{
    timespec ts{};
#ifdef CLOCK_BOOTTIME
    clock_gettime(CLOCK_BOOTTIME, &ts);
#else
    clock_gettime(CLOCK_MONOTONIC, &ts);
#endif
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
}

namespace common_utils_detail {

inline volatile sig_atomic_t *&stop_flag()
{
    static volatile sig_atomic_t *flag = nullptr;
    return flag;
}

inline void stop_signal_handler(int)
{
    if (stop_flag() != nullptr) *stop_flag() = 1;
}

}

inline void install_stop_signal_handlers(volatile sig_atomic_t *stop_flag)
{
    common_utils_detail::stop_flag() = stop_flag;
    signal(SIGINT, common_utils_detail::stop_signal_handler);
    signal(SIGTERM, common_utils_detail::stop_signal_handler);
}

#endif // COMMON_UTILS_H
