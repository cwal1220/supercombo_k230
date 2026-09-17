#ifndef UTILS_TIME_H
#define UTILS_TIME_H

/* 런타임 공통 시계는 k230_now_ns 하나다(CLOCK_BOOTTIME). 프로세스 사이를
 * 건너가는 타임스탬프는 전부 이 시계로 찍는다. */

#include <cstdint>
#include <sys/time.h>
#include <time.h>

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

inline uint64_t timeval_us(const timeval &tv)
{
    return static_cast<uint64_t>(tv.tv_sec) * 1000000ULL + tv.tv_usec;
}

/* ns 타임스탬프 freshness 공통 판정. 0(미설정)과 미래 타임스탬프를 모두 stale로 본다. */
inline bool timestamp_fresh_ns(uint64_t timestamp_ns, uint64_t now_ns, uint64_t max_age_ns)
{
    return timestamp_ns != 0 && now_ns >= timestamp_ns && now_ns - timestamp_ns <= max_age_ns;
}

/* CAN 신호 freshness 공통 판정. 미수신(음수)과 미래 타임스탬프를 모두
 * stale로 본다. */
inline bool signal_time_fresh(double timestamp_s, double now_s, double timeout_s)
{
    return timestamp_s >= 0.0 && now_s >= timestamp_s &&
           now_s - timestamp_s <= timeout_s;
}

#endif
