#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <string>

#include "common_utils.h"

constexpr unsigned kDefaultSensorWidth = 1920;
constexpr unsigned kDefaultSensorHeight = 1080;
/* AI 캡처 기본 해상도. 720p는 supercombo 학습 카메라(17.9 px/°)를 넘는
 * 각해상도(21 px/°)를 주고, 실측(2026-08-15)에서 20fps 유지 + modeld
 * 처리시간 +2.2ms(5%)만 증가했다. SUPERCOMBO_NV12_WIDTH/HEIGHT로 오버라이드. */
constexpr unsigned kDefaultAiWidth = 1280;
constexpr unsigned kDefaultAiHeight = 720;
constexpr unsigned kDefaultModelWidth = 512;
constexpr unsigned kDefaultModelHeight = 256;
constexpr float kDefaultModelFx = 910.0f;
constexpr float kDefaultModelFy = 910.0f;
constexpr float kDefaultModelCx = 256.0f;
constexpr float kDefaultModelCy = 47.6f;
constexpr float kK230CameraFx = 1583.3981f;
constexpr float kK230CameraFy = 1583.7622f;
constexpr float kK230CameraCx = 954.9441f;
constexpr float kK230CameraCy = 545.1774f;
constexpr float default_input_warp_fx(unsigned source_width)
{
    return kK230CameraFx * static_cast<float>(source_width) /
        static_cast<float>(kDefaultSensorWidth);
}
constexpr float default_input_warp_fy(unsigned source_height)
{
    return kK230CameraFy * static_cast<float>(source_height) /
        static_cast<float>(kDefaultSensorHeight);
}
constexpr float default_input_warp_cx(unsigned source_width)
{
    return kK230CameraCx * static_cast<float>(source_width) /
        static_cast<float>(kDefaultSensorWidth);
}
constexpr float default_input_warp_cy(unsigned source_height)
{
    return kK230CameraCy * static_cast<float>(source_height) /
        static_cast<float>(kDefaultSensorHeight);
}
constexpr float kDefaultInputWarpFx = default_input_warp_fx(kDefaultAiWidth);
constexpr float kDefaultInputWarpFy = default_input_warp_fy(kDefaultAiHeight);
constexpr float kDefaultInputWarpCx = default_input_warp_cx(kDefaultAiWidth);
constexpr float kDefaultInputWarpCy = default_input_warp_cy(kDefaultAiHeight);

struct AppConfig {
    std::string kmodel_path;
    int debug_mode = 1;

    unsigned nv12_width = kDefaultAiWidth;
    unsigned nv12_height = kDefaultAiHeight;
    unsigned nv12_crop_x = 0;
    unsigned nv12_crop_y = 0;
    unsigned nv12_crop_width = kDefaultSensorWidth;
    unsigned nv12_crop_height = kDefaultSensorHeight;
    unsigned model_fps = 20;
    unsigned max_frames = 0;

    std::string replay_nv12_path;

    bool calibration_auto = true;
    bool manual_calibration = false;
    float manual_roll = 0.0f;
    float manual_pitch = 0.0f;
    float manual_yaw = 0.0f;
    bool log_calibration = false;
    bool profile = false;

    float input_warp_fx = kDefaultInputWarpFx;
    float input_warp_fy = kDefaultInputWarpFy;
    float input_warp_cx = kDefaultInputWarpCx;
    float input_warp_cy = kDefaultInputWarpCy;
    float input_warp_height = 1.22f;

    static AppConfig from_env(int argc, char *argv[]);
    static AppConfig from_env_defaults();
    static std::string usage(const char *program_name);

    bool replay_enabled() const { return !replay_nv12_path.empty(); }
};

#endif
