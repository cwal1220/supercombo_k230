#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

/* /proc CPU/메모리/저장소, Canaan 온도 레지스터, 네트워크 상태를 읽어
 * OverlayHudState에 채운다. */

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <sys/statvfs.h>

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "overlay_renderer.h"

class SystemMonitor {
public:
    void sample(OverlayHudState *hud)
    {
        if (!hud) return;
        sample_cpu(&hud->cpu_percent);
        sample_memory(&hud->memory_percent);
        sample_storage(&hud->storage_percent);
        sample_temperature(&hud->cpu_temp_c);
        sample_network(hud);
    }

private:
    static float canaan_temperature_c(float raw_value)
    {
        const unsigned register_value = static_cast<unsigned>(raw_value);
        const float data = static_cast<float>(register_value & 0x0fffU);
        return ((((1.01472e-10f * data - 1.10063e-6f) * data + 4.36150e-3f) * data -
                 7.10128f) * data + 3565.87f);
    }

    void sample_cpu(float *percent)
    {
        FILE *file = std::fopen("/proc/stat", "r");
        if (!file) return;
        unsigned long long user = 0, nice = 0, system = 0, idle = 0;
        unsigned long long iowait = 0, irq = 0, softirq = 0, steal = 0;
        const int count = std::fscanf(file, "cpu %llu %llu %llu %llu %llu %llu %llu %llu",
                                      &user, &nice, &system, &idle, &iowait,
                                      &irq, &softirq, &steal);
        std::fclose(file);
        if (count < 4) return;

        const uint64_t idle_total = idle + iowait;
        const uint64_t total = user + nice + system + idle + iowait + irq + softirq + steal;
        if (previous_total_ != 0 && total > previous_total_) {
            const uint64_t total_delta = total - previous_total_;
            const uint64_t idle_delta = idle_total - previous_idle_;
            *percent = static_cast<float>(100.0 *
                (1.0 - static_cast<double>(std::min(idle_delta, total_delta)) / total_delta));
        }
        previous_total_ = total;
        previous_idle_ = idle_total;
    }

    void sample_memory(float *percent)
    {
        FILE *file = std::fopen("/proc/meminfo", "r");
        if (!file) return;
        unsigned long long total_kb = 0;
        unsigned long long available_kb = 0;
        char line[128];
        while (std::fgets(line, sizeof(line), file)) {
            std::sscanf(line, "MemTotal: %llu kB", &total_kb);
            std::sscanf(line, "MemAvailable: %llu kB", &available_kb);
        }
        std::fclose(file);
        if (total_kb > 0) {
            *percent = static_cast<float>(100.0 *
                (1.0 - static_cast<double>(std::min(available_kb, total_kb)) / total_kb));
        }
    }

    void sample_storage(float *percent)
    {
        struct statvfs space {};
        if (statvfs("/", &space) != 0 || space.f_blocks == 0) return;
        const uint64_t available = space.f_bavail;
        const uint64_t total = space.f_blocks;
        *percent = static_cast<float>(100.0 *
            (1.0 - static_cast<double>(std::min(available, total)) / total));
    }

    void sample_temperature(float *temperature_c)
    {
        float maximum = 0.0f;
        for (int i = 0; i < 16; ++i) {
            char path[96];
            std::snprintf(path, sizeof(path), "/sys/class/thermal/thermal_zone%d/temp", i);
            FILE *file = std::fopen(path, "r");
            if (!file) continue;
            float value = 0.0f;
            const bool read = std::fscanf(file, "%f", &value) == 1;
            std::fclose(file);
            if (!read) continue;

            char type_path[96];
            std::snprintf(type_path, sizeof(type_path),
                          "/sys/class/thermal/thermal_zone%d/type", i);
            FILE *type_file = std::fopen(type_path, "r");
            char type[64] = {};
            const bool type_read = type_file && std::fgets(type, sizeof(type), type_file);
            if (type_file) std::fclose(type_file);

            if (type_read && std::strncmp(type, "canaan_thermal_zone", 19) == 0 &&
                value >= 4096.0f) {
                value = canaan_temperature_c(value);
            } else if (value > 1000.0f) {
                value /= 1000.0f;
            }
            if (value > 0.0f && value < 200.0f) maximum = std::max(maximum, value);
        }
        *temperature_c = maximum;
    }

    void sample_network(OverlayHudState *hud)
    {
        hud->network_connected = false;
        hud->wifi_signal_dbm = 0;
        hud->network_interface[0] = '\0';
        hud->network_ipv4[0] = '\0';

        ifaddrs *addresses = nullptr;
        if (getifaddrs(&addresses) != 0) return;

        int best_score = -1;
        for (const ifaddrs *address = addresses; address; address = address->ifa_next) {
            if (!address->ifa_addr || address->ifa_addr->sa_family != AF_INET) continue;
            if ((address->ifa_flags & IFF_UP) == 0 ||
                (address->ifa_flags & IFF_LOOPBACK) != 0) {
                continue;
            }

            const char *name = address->ifa_name ? address->ifa_name : "";
            const int score = std::strcmp(name, "wlan0") == 0 ? 3 :
                              (std::strncmp(name, "wlan", 4) == 0 ? 2 : 1);
            if (score <= best_score) continue;

            char ipv4[INET_ADDRSTRLEN] = {};
            const sockaddr_in *socket_address =
                reinterpret_cast<const sockaddr_in *>(address->ifa_addr);
            if (!inet_ntop(AF_INET, &socket_address->sin_addr,
                           ipv4, sizeof(ipv4))) {
                continue;
            }

            best_score = score;
            hud->network_connected = true;
            std::snprintf(hud->network_interface, sizeof(hud->network_interface),
                          "%s", name);
            std::snprintf(hud->network_ipv4, sizeof(hud->network_ipv4),
                          "%s", ipv4);
        }
        freeifaddrs(addresses);

        if (!hud->network_connected ||
            std::strncmp(hud->network_interface, "wlan", 4) != 0) {
            return;
        }

        FILE *file = std::fopen("/proc/net/wireless", "r");
        if (!file) return;
        char line[160];
        while (std::fgets(line, sizeof(line), file)) {
            char interface_name[16] = {};
            unsigned status = 0;
            float link = 0.0f;
            float level = 0.0f;
            float noise = 0.0f;
            if (std::sscanf(line, " %15[^:]: %x %f %f %f",
                            interface_name, &status, &link, &level, &noise) == 5 &&
                std::strcmp(interface_name, hud->network_interface) == 0) {
                hud->wifi_signal_dbm = static_cast<int>(std::lround(level));
                break;
            }
        }
        std::fclose(file);
    }

    uint64_t previous_total_ = 0;
    uint64_t previous_idle_ = 0;
};

#endif  // SYSTEM_MONITOR_H
