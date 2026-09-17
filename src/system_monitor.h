#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

/* /proc CPU/메모리/저장소, Canaan 온도 레지스터, 네트워크 상태를 읽어
 * OverlayHudState에 채운다. k230_overlayd가 1 Hz로 호출한다. */

#include "overlay_state.h"

#include <cstdint>

class SystemMonitor {
public:
    void sample(OverlayHudState *hud);

private:
    static float canaan_temperature_c(float raw_value);
    void sample_cpu(float *percent);
    void sample_memory(float *percent);
    void sample_storage(float *percent);
    void sample_temperature(float *temperature_c);
    void sample_network(OverlayHudState *hud);

    uint64_t previous_total_ = 0;
    uint64_t previous_idle_ = 0;
};

#endif  // SYSTEM_MONITOR_H
