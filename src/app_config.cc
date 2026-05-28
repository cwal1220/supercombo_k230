#include "app_config.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace {

constexpr float kPi = 3.14159265358979323846f;

std::string env_string(const char *name)
{
    const char *value = std::getenv(name);
    return value && value[0] != '\0' ? std::string(value) : std::string();
}

ProjectionMode env_projection_mode()
{
    const char *value = std::getenv("SUPERCOMBO_PROJECTION_MODE");
    if (!value || value[0] == '\0') return ProjectionMode::Legacy;
    if (std::strcmp(value, "openpilot") == 0) return ProjectionMode::Openpilot;
    if (std::strcmp(value, "legacy") == 0) return ProjectionMode::Legacy;
    return ProjectionMode::Legacy;
}

} // namespace

bool env_enabled(const char *name)
{
    const char *value = std::getenv(name);
    return value && value[0] != '\0' && std::strcmp(value, "0") != 0;
}

bool env_enabled_default(const char *name, bool default_value)
{
    const char *value = std::getenv(name);
    if (!value) return default_value;
    return value[0] != '\0' && std::strcmp(value, "0") != 0;
}

bool env_present(const char *name)
{
    return std::getenv(name) != nullptr;
}

unsigned env_unsigned(const char *name, unsigned default_value)
{
    const char *value = std::getenv(name);
    if (!value || value[0] == '\0') return default_value;
    char *end = nullptr;
    const unsigned long parsed = std::strtoul(value, &end, 10);
    return end == value ? default_value : static_cast<unsigned>(parsed);
}

float env_float(const char *name, float default_value)
{
    const char *value = std::getenv(name);
    if (!value || value[0] == '\0') return default_value;
    char *end = nullptr;
    const float parsed = std::strtof(value, &end);
    return end == value ? default_value : parsed;
}

float deg_to_rad(float deg)
{
    return deg * kPi / 180.0f;
}

float rad_to_deg(float rad)
{
    return rad * 180.0f / kPi;
}

const char *projection_mode_name(ProjectionMode mode)
{
    switch (mode) {
    case ProjectionMode::Openpilot:
        return "openpilot";
    case ProjectionMode::Legacy:
        return "legacy";
    }
    return "unknown";
}

std::string AppConfig::usage(const char *program_name)
{
    return std::string("Usage: ") + (program_name ? program_name : "supercombo.elf") +
        " <supercombo.kmodel> [debug_mode]";
}

AppConfig AppConfig::from_env_defaults(const char *program_name)
{
    AppConfig config;
    config.program_name = program_name ? program_name : "supercombo.elf";

    config.nv12_width = env_unsigned("SUPERCOMBO_NV12_WIDTH", config.nv12_width);
    config.nv12_height = env_unsigned("SUPERCOMBO_NV12_HEIGHT", config.nv12_height);
    config.nv12_crop_x = env_unsigned("SUPERCOMBO_NV12_CROP_X", config.nv12_crop_x);
    config.nv12_crop_y = env_unsigned("SUPERCOMBO_NV12_CROP_Y", config.nv12_crop_y);
    config.nv12_crop_width = env_unsigned("SUPERCOMBO_NV12_CROP_WIDTH", config.nv12_crop_width);
    config.nv12_crop_height = env_unsigned("SUPERCOMBO_NV12_CROP_HEIGHT", config.nv12_crop_height);
    config.ai_start_preview_frames = std::max(1u, env_unsigned("SUPERCOMBO_AI_START_PREVIEW_FRAMES",
                                                              config.ai_start_preview_frames));
    config.max_frames = env_unsigned("SUPERCOMBO_MAX_FRAMES", 0);

    config.replay_nv12_path = env_string("SUPERCOMBO_REPLAY_NV12");
    config.raw_dump_path = env_string("SUPERCOMBO_DUMP_RAW");

    config.manual_calibration = env_present("SUPERCOMBO_CALIB_ROLL_DEG") ||
        env_present("SUPERCOMBO_CALIB_PITCH_DEG") ||
        env_present("SUPERCOMBO_CALIB_YAW_DEG");
    config.calibration_auto = env_enabled_default("SUPERCOMBO_CALIB_AUTO", true);
    config.manual_roll = deg_to_rad(env_float("SUPERCOMBO_CALIB_ROLL_DEG", 0.0f));
    config.manual_pitch = deg_to_rad(env_float("SUPERCOMBO_CALIB_PITCH_DEG", 0.0f));
    config.manual_yaw = deg_to_rad(env_float("SUPERCOMBO_CALIB_YAW_DEG", 0.0f));
    config.log_calibration = env_enabled("SUPERCOMBO_LOG_CALIB");
    config.log_pose = env_enabled("SUPERCOMBO_LOG_POSE");
    config.log_control = env_enabled("SUPERCOMBO_LOG_CONTROL");
    config.profile = env_enabled("SUPERCOMBO_PROFILE");

    config.projection_mode = env_projection_mode();
    config.draw_lead = env_enabled_default("SUPERCOMBO_DRAW_LEAD", true);
    config.lead_prob_threshold = env_float("SUPERCOMBO_LEAD_PROB_THRESHOLD", config.lead_prob_threshold);
    config.lead_time_idx = static_cast<int>(std::min(2u, env_unsigned("SUPERCOMBO_LEAD_TIME_IDX", 0)));

    return config;
}

AppConfig AppConfig::from_env(int argc, char *argv[])
{
    if (argc < 2 || argc > 3)
        throw std::invalid_argument(usage(argc > 0 ? argv[0] : "supercombo.elf"));

    AppConfig config = from_env_defaults(argv[0]);
    config.kmodel_path = argv[1];
    config.debug_mode = argc >= 3 ? std::atoi(argv[2]) : 1;
    return config;
}
