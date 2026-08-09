#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "hyundai_can.h"

struct TimedCanFrame {
  uint64_t timestamp_us = 0;
  CanFrame frame;
};

class CanReplaySource {
public:
  // K230CAN1 binary fixture를 읽고 구조와 시간 순서를 검증한다.
  void open(const std::string &path);

  // elapsed time까지 도착한 CAN frame을 반환한다.
  size_t poll(double elapsed_s, std::vector<CanFrame> *frames);

  bool valid() const { return !records_.empty(); }
  bool finished() const { return next_record_ >= records_.size(); }
  size_t total_frames() const { return records_.size(); }
  size_t emitted_frames() const { return next_record_; }
  double duration_s() const;

private:
  std::vector<TimedCanFrame> records_;
  size_t next_record_ = 0;
};
