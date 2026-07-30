#include "alert_sound_player.h"

#include <alsa/asoundlib.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace {

constexpr unsigned kSampleRate = 48000;
constexpr double kPi = 3.14159265358979323846;

bool sound_enabled() {
  const char *value = std::getenv("K230_ALERT_SOUND");
  return !value || (std::strcmp(value, "0") != 0 &&
                    std::strcmp(value, "false") != 0 &&
                    std::strcmp(value, "FALSE") != 0);
}

void append_tone(std::vector<int16_t> *samples, float frequency_hz,
                 float duration_s, float volume) {
  const size_t count = static_cast<size_t>(duration_s * kSampleRate);
  const size_t ramp = std::min<size_t>(count / 2, kSampleRate / 100);
  const size_t start = samples->size();
  samples->resize(start + count);
  for (size_t i = 0; i < count; ++i) {
    float envelope = 1.0f;
    if (i < ramp) envelope = static_cast<float>(i) / ramp;
    if (count - i <= ramp)
      envelope = std::min(envelope, static_cast<float>(count - i) / ramp);
    const double phase =
        2.0 * kPi * frequency_hz * static_cast<double>(i) / kSampleRate;
    (*samples)[start + i] =
        static_cast<int16_t>(std::sin(phase) * 32767.0 * volume * envelope);
  }
}

void append_silence(std::vector<int16_t> *samples, float duration_s) {
  samples->resize(samples->size() +
                  static_cast<size_t>(duration_s * kSampleRate), 0);
}

std::vector<int16_t> make_chime(DepartureAlertType type) {
  std::vector<int16_t> samples;
  samples.reserve(kSampleRate / 2);
  if (type == DepartureAlertType::lead_departed) {
    append_tone(&samples, 880.0f, 0.11f, 0.35f);
    append_silence(&samples, 0.04f);
    append_tone(&samples, 1174.7f, 0.14f, 0.35f);
  } else {
    append_tone(&samples, 784.0f, 0.11f, 0.35f);
    append_silence(&samples, 0.04f);
    append_tone(&samples, 1046.5f, 0.16f, 0.35f);
  }
  return samples;
}

}  // namespace

AlertSoundPlayer::AlertSoundPlayer()
    : enabled_(sound_enabled()), thread_(&AlertSoundPlayer::run, this) {}

AlertSoundPlayer::~AlertSoundPlayer() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  condition_.notify_one();
  if (thread_.joinable()) thread_.join();
}

void AlertSoundPlayer::play(DepartureAlertType type, uint32_t event_id) {
  if (!enabled_ || type == DepartureAlertType::none || event_id == 0) return;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_type_ = type;
    pending_event_id_ = event_id;
    pending_ = true;
  }
  condition_.notify_one();
}

void AlertSoundPlayer::run() {
  while (true) {
    DepartureAlertType type = DepartureAlertType::none;
    uint32_t event_id = 0;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [&] { return stop_ || pending_; });
      if (stop_) return;
      type = pending_type_;
      event_id = pending_event_id_;
      pending_ = false;
    }

    if (!play_tone(type) && !error_reported_) {
      error_reported_ = true;
      std::fprintf(stderr,
                   "k230_overlay: alert sound unavailable, continuing with LCD "
                   "alerts (event=%u)\n",
                   event_id);
    }
  }
}

bool AlertSoundPlayer::play_tone(DepartureAlertType type) {
  const char *device = std::getenv("K230_ALERT_PCM");
  if (!device || device[0] == '\0') device = "default";

  snd_pcm_t *pcm = nullptr;
  if (snd_pcm_open(&pcm, device, SND_PCM_STREAM_PLAYBACK, 0) < 0)
    return false;
  const int configure_result =
      snd_pcm_set_params(pcm, SND_PCM_FORMAT_S16_LE,
                         SND_PCM_ACCESS_RW_INTERLEAVED, 1, kSampleRate, 1,
                         500000);
  if (configure_result < 0) {
    snd_pcm_close(pcm);
    return false;
  }

  const std::vector<int16_t> samples = make_chime(type);
  size_t offset = 0;
  while (offset < samples.size()) {
    const snd_pcm_sframes_t written =
        snd_pcm_writei(pcm, samples.data() + offset, samples.size() - offset);
    if (written < 0) {
      if (snd_pcm_recover(pcm, static_cast<int>(written), 1) < 0) {
        snd_pcm_close(pcm);
        return false;
      }
      continue;
    }
    offset += static_cast<size_t>(written);
  }
  snd_pcm_drain(pcm);
  snd_pcm_close(pcm);
  return true;
}
