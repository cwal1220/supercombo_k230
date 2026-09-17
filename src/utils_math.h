#ifndef UTILS_MATH_H
#define UTILS_MATH_H

#include <algorithm>
#include <cmath>
#include <initializer_list>

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

#endif
