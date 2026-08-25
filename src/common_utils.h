#ifndef COMMON_UTILS_H
#define COMMON_UTILS_H

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <cstdlib>
#include <csignal>
#include <cstring>
#include <string>
#include <sys/time.h>
#include <time.h>

/* 환경변수 불리언 공통 규약. 미설정이거나 빈 값이면 default_value,
 * "0"/"false"/"no"/"off"/"n"(대소문자 무시)이면 false, 나머지는 true. */
inline bool env_flag(const char *name, bool default_value = false)
{
    const char *value = std::getenv(name);
    if (!value || value[0] == '\0') return default_value;
    std::string lowered(value);
    for (char &character : lowered)
        character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
    return !(lowered == "0" || lowered == "false" || lowered == "no" ||
             lowered == "off" || lowered == "n");
}

/* CAN 신호 freshness 공통 판정. 미수신(음수)과 미래 타임스탬프를 모두
 * stale로 본다. */
inline bool signal_time_fresh(double timestamp_s, double now_s, double timeout_s)
{
    return timestamp_s >= 0.0 && now_s >= timestamp_s &&
           now_s - timestamp_s <= timeout_s;
}

inline float clamp_float(float value, float lo, float hi)
{
    if (!std::isfinite(value)) return lo;
    return std::min(std::max(value, lo), hi);
}

// openpilot interp: 단조 증가 xp에 대한 구간 선형 보간. 범위 밖은 끝값.
inline float interp(float x, std::initializer_list<float> xp,
                    std::initializer_list<float> fp) {
  const auto xs = xp.begin();
  const auto fs = fp.begin();
  const size_t n = xp.size();
  if (n == 0 || fp.size() != n) return 0.0f;
  if (x <= xs[0]) return fs[0];
  if (x >= xs[n - 1]) return fs[n - 1];
  for (size_t idx = 1; idx < n; ++idx) {
    if (x <= xs[idx]) {
      const float low_x = xs[idx - 1];
      const float high_x = xs[idx];
      const float low_y = fs[idx - 1];
      const float high_y = fs[idx];
      if (high_x - low_x < 1e-6f && low_x - high_x < 1e-6f) return high_y;
      return low_y + (x - low_x) * (high_y - low_y) / (high_x - low_x);
    }
  }
  return fs[n - 1];
}

inline float deg_to_rad(float deg) {
  return deg * 0.017453292519943295f;
}

inline float rad_to_deg(float rad) {
  return rad * 57.29577951308232f;
}

inline bool env_present(const char *name) {
  return std::getenv(name) != nullptr;
}

inline unsigned env_unsigned(const char *name, unsigned default_value) {
  const char *value = std::getenv(name);
  if (!value || value[0] == '\0') return default_value;
  char *end = nullptr;
  const unsigned long parsed = std::strtoul(value, &end, 10);
  return end == value ? default_value : static_cast<unsigned>(parsed);
}

inline float env_float(const char *name, float default_value) {
  const char *value = std::getenv(name);
  if (!value || value[0] == '\0') return default_value;
  char *end = nullptr;
  const float parsed = std::strtof(value, &end);
  return end == value ? default_value : parsed;
}


inline std::string env_string(const char *name, const char *fallback = "") {
  const char *value = std::getenv(name);
  return value && value[0] ? value : fallback;
}

// params 디렉토리 경로 결정 (K230_PARAMS_DIR 재정의 가능)
inline std::string k230_params_dir() {
  const char *value = std::getenv("K230_PARAMS_DIR");
  return value && value[0] != '\0' ? std::string(value) : std::string("params");
}

inline std::string k230_param_path(const char *name) {
  return k230_params_dir() + "/" + (name ? name : "");
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
