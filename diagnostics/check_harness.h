#pragma once

#include <cmath>
#include <cstdio>
#include <exception>
#include <stdexcept>

/* 호스트 자체 검사 공용 하네스. 검사는 require로 던지고 main은 run_checks 하나로 끝낸다. */

inline void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

/* 허용 오차 포함(<=). */
inline bool near(float value, float expected, float tolerance = 1e-6f) {
  return std::fabs(value - expected) <= tolerance;
}

/* body가 던지지 않으면 ok_line(nullptr면 생략)을 찍고 0, 던지면 메시지를 stderr에 찍고 1. */
template <class F>
int run_checks(const char *ok_line, F &&body) {
  try {
    body();
  } catch (const std::exception &error) {
    std::fprintf(stderr, "%s\n", error.what());
    return 1;
  }
  if (ok_line) std::puts(ok_line);
  return 0;
}
