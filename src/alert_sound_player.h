#pragma once

#include "departure_alert.h"

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

class AlertSoundPlayer {
public:
  AlertSoundPlayer();
  ~AlertSoundPlayer();

  void play(DepartureAlertType type, uint32_t event_id);

private:
  void run();
  bool play_tone(DepartureAlertType type);

  bool enabled_ = true;
  bool stop_ = false;
  bool pending_ = false;
  bool error_reported_ = false;
  DepartureAlertType pending_type_ = DepartureAlertType::none;
  uint32_t pending_event_id_ = 0;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::thread thread_;
};
