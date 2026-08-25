#include "app_config.h"

#include "common_utils.h"

#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace {

constexpr float kPi = 3.14159265358979323846f;

} // namespace

std::string AppConfig::usage(const char *program_name)
{
    return std::string("Usage: ") + (program_name ? program_name : "k230_modeld") +
        " <supercombo.kmodel> [debug_mode]";
}

AppConfig AppConfig::from_env_defaults()
{
    AppConfig config;


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
