#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "setting.h"

#include <string>

enum class ProjectionMode {
    Openpilot = 0,
    Legacy = 1,
};

struct AppConfig {
    std::string program_name;
    std::string kmodel_path;
    int debug_mode = 1;

    unsigned nv12_width = 512;
    unsigned nv12_height = 256;
    unsigned nv12_crop_x = 0;
    unsigned nv12_crop_y = 0;
    unsigned nv12_crop_width = SENSOR_WIDTH;
    unsigned nv12_crop_height = SENSOR_HEIGHT;
    unsigned max_frames = 0;

    std::string replay_nv12_path;

    bool calibration_auto = true;
    bool manual_calibration = false;
    float manual_roll = 0.0f;
    float manual_pitch = 0.0f;
    float manual_yaw = 0.0f;
    bool log_calibration = false;
    bool profile = false;

    ProjectionMode projection_mode = ProjectionMode::Legacy;

    static AppConfig from_env(int argc, char *argv[]);
    static AppConfig from_env_defaults(const char *program_name);
    static std::string usage(const char *program_name);

    bool replay_enabled() const { return !replay_nv12_path.empty(); }
};

bool env_enabled(const char *name);
bool env_enabled_default(const char *name, bool default_value);
bool env_present(const char *name);
unsigned env_unsigned(const char *name, unsigned default_value);
float env_float(const char *name, float default_value);
float deg_to_rad(float deg);
float rad_to_deg(float rad);
const char *projection_mode_name(ProjectionMode mode);

#endif
