#include "app_config.h"

#include "common_utils.h"

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

} // namespace

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

float env_float_prefer(const char *primary, const char *fallback, float default_value)
{
    if (env_present(primary)) return env_float(primary, default_value);
    return env_float(fallback, default_value);
}

float deg_to_rad(float deg)
{
    return deg * kPi / 180.0f;
}

float rad_to_deg(float rad)
{
    return rad * 180.0f / kPi;
}

std::string AppConfig::usage(const char *program_name)
{
    return std::string("Usage: ") + (program_name ? program_name : "k230_modeld") +
        " <supercombo.kmodel> [debug_mode]";
}

AppConfig AppConfig::from_env_defaults()
{
    AppConfig config;

    config.nv12_width = env_unsigned("SUPERCOMBO_NV12_WIDTH", config.nv12_width);
    config.nv12_height = env_unsigned("SUPERCOMBO_NV12_HEIGHT", config.nv12_height);
    config.nv12_crop_x = env_unsigned("SUPERCOMBO_NV12_CROP_X", config.nv12_crop_x);
    config.nv12_crop_y = env_unsigned("SUPERCOMBO_NV12_CROP_Y", config.nv12_crop_y);
    config.nv12_crop_width = env_unsigned("SUPERCOMBO_NV12_CROP_WIDTH", config.nv12_crop_width);
    config.nv12_crop_height = env_unsigned("SUPERCOMBO_NV12_CROP_HEIGHT", config.nv12_crop_height);
    config.model_fps = env_unsigned("SUPERCOMBO_MODEL_FPS", config.model_fps);

    config.input_warp_fx = default_input_warp_fx(config.nv12_width);
    config.input_warp_fy = default_input_warp_fy(config.nv12_height);
    config.input_warp_cx = default_input_warp_cx(config.nv12_width);
    config.input_warp_cy = default_input_warp_cy(config.nv12_height);

    config.max_frames = env_unsigned("SUPERCOMBO_MAX_FRAMES", 0);

    config.replay_nv12_path = env_string("SUPERCOMBO_REPLAY_NV12");

    config.manual_calibration = env_present("SUPERCOMBO_CALIB_ROLL_DEG") ||
        env_present("SUPERCOMBO_CALIB_PITCH_DEG") ||
        env_present("SUPERCOMBO_CALIB_YAW_DEG");
    config.calibration_auto = env_flag("SUPERCOMBO_CALIB_AUTO", true);
    config.manual_roll = deg_to_rad(env_float("SUPERCOMBO_CALIB_ROLL_DEG", 0.0f));
    config.manual_pitch = deg_to_rad(env_float("SUPERCOMBO_CALIB_PITCH_DEG", 0.0f));
    config.manual_yaw = deg_to_rad(env_float("SUPERCOMBO_CALIB_YAW_DEG", 0.0f));
    config.log_calibration = env_flag("SUPERCOMBO_LOG_CALIB");
    config.profile = env_flag("SUPERCOMBO_PROFILE");

    config.input_warp_fx = env_float("SUPERCOMBO_INPUT_WARP_FX", config.input_warp_fx);
    config.input_warp_fy = env_float("SUPERCOMBO_INPUT_WARP_FY", config.input_warp_fy);
    config.input_warp_cx = env_float("SUPERCOMBO_INPUT_WARP_CX", config.input_warp_cx);
    config.input_warp_cy = env_float("SUPERCOMBO_INPUT_WARP_CY", config.input_warp_cy);
    config.input_warp_height = env_float("SUPERCOMBO_INPUT_WARP_HEIGHT", config.input_warp_height);
    config.input_warp_roll = deg_to_rad(env_float_prefer("SUPERCOMBO_INPUT_WARP_ROLL_DEG", "SUPERCOMBO_CALIB_ROLL_DEG", 0.0f));
    config.input_warp_pitch = deg_to_rad(env_float_prefer("SUPERCOMBO_INPUT_WARP_PITCH_DEG", "SUPERCOMBO_CALIB_PITCH_DEG", 0.0f));
    config.input_warp_yaw = deg_to_rad(env_float_prefer("SUPERCOMBO_INPUT_WARP_YAW_DEG", "SUPERCOMBO_CALIB_YAW_DEG", 0.0f));

    return config;
}

AppConfig AppConfig::from_env(int argc, char *argv[])
{
    if (argc < 2 || argc > 3)
        throw std::invalid_argument(usage(argc > 0 ? argv[0] : "k230_modeld"));

    AppConfig config = from_env_defaults();
    config.kmodel_path = argv[1];
    config.debug_mode = argc >= 3 ? std::atoi(argv[2]) : 1;
    return config;
}
