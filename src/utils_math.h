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

// row-major 3x3 곱과 3x3·3x4 곱. 결합 순서(k 내부 누적)가 세 호출자 모두 같았다.
inline void matmul3(const float *a, const float *b, float *out)
{
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < 3; ++k)
                sum += a[r * 3 + k] * b[k * 3 + c];
            out[r * 3 + c] = sum;
        }
    }
}

inline void matmul34(const float *a3, const float *b34, float *out34)
{
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < 3; ++k)
                sum += a3[r * 3 + k] * b34[k * 4 + c];
            out34[r * 4 + c] = sum;
        }
    }
}

inline float deg_to_rad(float deg) {
  return deg * 0.017453292519943295f;
}

inline float rad_to_deg(float rad) {
  return rad * 57.29577951308232f;
}

#endif
