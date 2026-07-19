#include "projection.h"

#include <cmath>

namespace {

void rotation_from_rpy(float roll, float pitch, float yaw, float *rot)
{
    const float cr = std::cos(roll);
    const float sr = std::sin(roll);
    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);
    const float cy = std::cos(yaw);
    const float sy = std::sin(yaw);

    rot[0] = cy * cp;
    rot[1] = cy * sp * sr - sy * cr;
    rot[2] = cy * sp * cr + sy * sr;
    rot[3] = sy * cp;
    rot[4] = sy * sp * sr + cy * cr;
    rot[5] = sy * sp * cr - cy * sr;
    rot[6] = -sp;
    rot[7] = cp * sr;
    rot[8] = cp * cr;
}

} // namespace

ProjectionState make_projection_state(float roll, float pitch, float yaw)
{
    ProjectionState state {};
    state.roll = roll;
    state.pitch = pitch;
    state.yaw = yaw;

    float rot[9] = {};
    rotation_from_rpy(roll, pitch, yaw, rot);

    // The renderer rotates model pixels by 180 degrees. Prepending the
    // road-axis flip makes that display transform cancel for every RPY.
    float device_from_calib[9] = {};
    for (int col = 0; col < 3; ++col) {
        device_from_calib[0 * 3 + col] = rot[0 * 3 + col];
        device_from_calib[1 * 3 + col] = -rot[1 * 3 + col];
        device_from_calib[2 * 3 + col] = -rot[2 * 3 + col];
    }

    // view_from_device = [[0,1,0],[0,0,1],[1,0,0]]
    for (int col = 0; col < 3; ++col) {
        state.view_from_calib[0 * 3 + col] = device_from_calib[1 * 3 + col];
        state.view_from_calib[1 * 3 + col] = device_from_calib[2 * 3 + col];
        state.view_from_calib[2 * 3 + col] = device_from_calib[0 * 3 + col];
    }

    return state;
}

bool project_point(const ProjectionState &projection, float x_forward, float y_left, float z_up,
                   int width, int height, int *px, int *py)
{
    if (x_forward < 0.5f || x_forward > 120.0f) return false;

    const float *m = projection.view_from_calib;
    const float vx = m[0] * x_forward + m[1] * y_left + m[2] * z_up;
    const float vy = m[3] * x_forward + m[4] * y_left + m[5] * z_up;
    const float vz = m[6] * x_forward + m[7] * y_left + m[8] * z_up;
    if (vz <= 0.1f) return false;

    const float landscape_w = 800.0f;
    const float landscape_h = 480.0f;
    const float fx = default_input_warp_fx(static_cast<unsigned>(landscape_w));
    const float fy = default_input_warp_fy(static_cast<unsigned>(landscape_h));
    const float calibrated_cx = default_input_warp_cx(static_cast<unsigned>(landscape_w));
    const float calibrated_cy = default_input_warp_cy(static_cast<unsigned>(landscape_h));
    const float cx = landscape_w - 1.0f - calibrated_cx;
    const float cy = landscape_h - 1.0f - calibrated_cy;

    const float u_land = fx * vx / vz + cx;
    const float v_land = fy * vy / vz + cy;

    if (width > height) {
        *px = static_cast<int>(std::round(u_land));
        *py = static_cast<int>(std::round(v_land));
    } else {
        *px = static_cast<int>(std::round(v_land));
        *py = static_cast<int>(std::round(landscape_w - 1.0f - u_land));
    }
    return *px > -200 && *px < width + 200 && *py > -200 && *py < height + 200;
}
