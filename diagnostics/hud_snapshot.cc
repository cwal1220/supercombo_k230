// HUD 스냅샷·타이밍 도구. 렌더러만 떼어 480x800 ARGB 버퍼에 그리고 시나리오별
// 프레임을 K230ARGB 파일로 저장한다. 호스트와 보드에서 같은 소스로 빌드한다.
//
//   hud_snapshot [--assets DIR] [--model model.bin] [--control control.bin]
//                [--iterations N] [--out PREFIX] [--landscape]
//
// model.bin / control.bin은 녹화 이벤트의 K230ModelState / K230ControlState
// 원본 바이트다(tools/ui/hud_tools.py inputs가 만든다). 없으면 합성 장면을 쓴다.
#include "k230_ipc.h"
#include "overlay_renderer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

uint64_t now_ns()
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool read_file(const std::string &path, void *dst, size_t size)
{
    FILE *file = std::fopen(path.c_str(), "rb");
    if (!file) return false;
    const size_t got = std::fread(dst, 1, size, file);
    std::fclose(file);
    return got == size;
}

/* K230ARGB: magic 8 B, u32 width, u32 height, BGRA 픽셀(행 우선). */
bool write_frame_file(const std::string &path, const OverlayTarget &target)
{
    FILE *file = std::fopen(path.c_str(), "wb");
    if (!file) return false;
    const char magic[8] = {'K', '2', '3', '0', 'A', 'R', 'G', 'B'};
    const uint32_t dims[2] = {target.width, target.height};
    bool ok = std::fwrite(magic, 1, 8, file) == 8 && std::fwrite(dims, 4, 2, file) == 2;
    for (uint32_t row = 0; ok && row < target.height; ++row) {
        const uint8_t *src = static_cast<const uint8_t *>(target.map) +
                             static_cast<size_t>(row) * target.stride;
        ok = std::fwrite(src, 4, target.width, file) == target.width;
    }
    std::fclose(file);
    return ok;
}

/* 직선 도로 합성 장면. 차선은 plan과 같은 지면에 놓이도록 z를 kModelHeight로 둔다. */
ParsedModelOutput synthetic_output()
{
    ParsedModelOutput output;
    output.valid = true;
    output.plan.valid = true;
    output.plan.probability = 0.9f;
    const float lane_y[4] = {-5.4f, -1.8f, 1.8f, 5.4f};
    const float lane_prob[4] = {0.35f, 0.95f, 0.95f, 0.35f};
    const float edge_y[2] = {-7.2f, 7.2f};
    for (int i = 0; i < kTrajectorySize; ++i) {
        const float x = model_x_idx(i);
        const float curve = 0.0006f * x * x;
        output.plan.points[i] = {x, curve, 0.0f};
        for (int lane = 0; lane < 4; ++lane) {
            output.lanes[lane].valid = true;
            output.lanes[lane].probability = lane_prob[lane];
            output.lanes[lane].points[i] = {x, lane_y[lane] + curve, kModelHeight};
        }
        for (int edge = 0; edge < 2; ++edge) {
            output.road_edges[edge].valid = true;
            output.road_edges[edge].std = 0.3f;
            output.road_edges[edge].points[i] = {x, edge_y[edge] + curve, kModelHeight};
        }
    }
    output.leads.valid = true;
    output.leads.global_probabilities[0] = 0.85f;
    output.leads.predictions[0].probabilities[0] = 0.85f;
    output.leads.predictions[0].points[0] = {42.0f, 0.4f, 16.0f, 0.0f};
    return output;
}

struct Scenario {
    const char *name;
    bool with_model;
    OverlayHudState hud;
};

void print_stats(const char *label, std::vector<double> values)
{
    std::sort(values.begin(), values.end());
    double sum = 0.0;
    for (double v : values) sum += v;
    std::printf("  %-8s mean=%.3f med=%.3f p90=%.3f max=%.3f ms\n", label,
                sum / values.size(), values[values.size() / 2],
                values[values.size() * 9 / 10], values.back());
}

} // namespace

int main(int argc, char **argv)
{
    std::string assets_dir = "assets/ui";
    std::string model_path;
    std::string control_path;
    std::string out_prefix = "hud_snapshot";
    int iterations = 50;
    bool landscape = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> const char * { return i + 1 < argc ? argv[++i] : ""; };
        if (arg == "--assets") assets_dir = next();
        else if (arg == "--model") model_path = next();
        else if (arg == "--control") control_path = next();
        else if (arg == "--iterations") iterations = std::max(1, std::atoi(next()));
        else if (arg == "--out") out_prefix = next();
        else if (arg == "--landscape") landscape = true;
        else {
            std::fprintf(stderr, "unknown argument %s\n", arg.c_str());
            return 2;
        }
    }

    K230ModelState model_state{};
    K230ControlState control_state{};
    const bool have_model = !model_path.empty() && read_file(model_path, &model_state, sizeof(model_state));
    const bool have_control = !control_path.empty() && read_file(control_path, &control_state, sizeof(control_state));
    if (!model_path.empty() && !have_model) std::fprintf(stderr, "cannot read %s\n", model_path.c_str());
    if (!control_path.empty() && !have_control) std::fprintf(stderr, "cannot read %s\n", control_path.c_str());

    ParsedModelOutput output = have_model ? k230_parsed_from_model_state(model_state) : synthetic_output();
    ProjectionState projection = have_model ? k230_projection_from_model_state(model_state)
                                            : make_projection_state(0.0f, 0.0f, 0.0f);

    OverlayHudState idle;
    idle.services_healthy = true;
    idle.network_connected = true;
    idle.cpu_percent = 43.0f;
    idle.memory_percent = 10.3f;
    idle.storage_percent = 4.6f;
    idle.cpu_temp_c = 48.0f;
    idle.preview_fps = 38.5f;
    idle.model_fps = 19.0f;
    idle.overlay_fps = 19.0f;
    idle.wifi_signal_dbm = -61;
    std::snprintf(idle.active_block, sizeof(idle.active_block), "control_stale");
    std::snprintf(idle.network_interface, sizeof(idle.network_interface), "wlan0");
    std::snprintf(idle.network_ipv4, sizeof(idle.network_ipv4), "192.168.219.111");

    OverlayHudState drive = idle;
    drive.panda_connected = true;
    drive.panda_healthy = true;
    drive.vehicle_fresh = true;
    drive.lateral_mode_available = true;
    drive.calibration_available = true;
    drive.calibration_status = 1;
    drive.calibration_valid_blocks = 12;
    drive.calibration_pitch_deg = -2.3f;
    if (have_control) {
        hud_apply_control_state(control_state, true, &drive);
    } else {
        drive.controller_enabled = drive.controller_engaged = drive.controller_active = true;
        drive.cruise_active = true;
        drive.gear = 5;
        drive.cluster_speed_kph = 65.0f;
        drive.ego_speed_kph = 64.0f;
        drive.cruise_command_speed_kph = 70.0f;
        drive.cruise_max_speed_kph = 70.0f;
        drive.steering_angle_deg = -3.0f;
        drive.desired_torque = 4;
        drive.apply_torque = 4;
        drive.driver_torque = 19;
        drive.tpms_valid = true;
        drive.tpms_pressure_fl = drive.tpms_pressure_fr = 36.0f;
        drive.tpms_pressure_rl = drive.tpms_pressure_rr = 35.0f;
    }

    OverlayHudState busy = drive;
    busy.left_blinker = true;
    busy.turn_signal_step = 10;
    busy.brake_hold = true;
    busy.green_light_alert_armed = true;
    std::snprintf(busy.engage_alert_message, sizeof(busy.engage_alert_message),
                  "UNABLE TO ENGAGE: LOW SPEED");

    OverlayHudState depart = drive;
    depart.departure_alert_type = DepartureAlertType::green_light;
    depart.cluster_speed_kph = 0.0f;
    depart.brake_hold = true;

    OverlayHudState fault = drive;
    fault.steering_fault = true;

    OverlayHudState standby = drive;
    standby.controller_engaged = standby.controller_active = false;
    standby.cruise_active = false;
    std::snprintf(standby.active_block, sizeof(standby.active_block), "stopped");
    standby.cluster_speed_kph = 12.0f;

    const std::vector<Scenario> scenarios = {
        {"idle", false, idle},
        {"standby", true, standby},
        {"drive", true, drive},
        {"busy", true, busy},
        {"depart", true, depart},
        {"fault", true, fault},
    };

    const uint32_t width = landscape ? 800 : 480;
    const uint32_t height = landscape ? 480 : 800;
    std::vector<uint32_t> storage(static_cast<size_t>(width) * height, 0);
    const OverlayTarget target{storage.data(), width, height, width * 4};

    OverlayRenderer renderer;
    std::printf("assets: %s (%s)\n", assets_dir.c_str(),
                renderer.load_assets(assets_dir) ? "loaded" : "missing");
    std::printf("inputs: model=%s control=%s target=%ux%u\n",
                have_model ? model_path.c_str() : "synthetic",
                have_control ? control_path.c_str() : "synthetic", width, height);

    for (const Scenario &scenario : scenarios) {
        const ParsedModelOutput &scene = scenario.with_model ? output : ParsedModelOutput{};
        std::vector<double> draw_ms;
        for (int i = 0; i < iterations; ++i) {
            const uint64_t t0 = now_ns();
            renderer.draw(target, scene, projection, scenario.hud, !landscape);
            draw_ms.push_back((now_ns() - t0) / 1e6);
        }
        std::printf("scenario %s (%d iters)\n", scenario.name, iterations);
        print_stats("draw", draw_ms);
        const std::string path = out_prefix + "_" + scenario.name + ".argb";
        if (!write_frame_file(path, target)) std::fprintf(stderr, "cannot write %s\n", path.c_str());
    }
    return 0;
}
