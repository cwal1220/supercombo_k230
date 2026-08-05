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

ProjectionState make_projection_state(float roll, float pitch, float yaw,
                                      const AppConfig *config)
{
    ProjectionState state {};
    state.roll = roll;
    state.pitch = pitch;
    state.yaw = yaw;
    if (config != nullptr) {
        state.camera_fx = config->input_warp_fx;
        state.camera_fy = config->input_warp_fy;
        state.camera_cx = config->input_warp_cx;
        state.camera_cy = config->input_warp_cy;
        state.camera_width = static_cast<float>(config->nv12_width);
        state.camera_height = static_cast<float>(config->nv12_height);
        state.dist_k1 = config->input_dist_k1;
        state.dist_k2 = config->input_dist_k2;
        state.dist_p1 = config->input_dist_p1;
        state.dist_p2 = config->input_dist_p2;
        state.dist_k3 = config->input_dist_k3;
    }

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

ProjectionState make_projection_state(float roll, float pitch, float yaw)
{
    return make_projection_state(roll, pitch, yaw, nullptr);
}

ProjectionState make_projection_state(float roll, float pitch, float yaw,
                                      const AppConfig &config)
{
    return make_projection_state(roll, pitch, yaw, &config);
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
    const float fx = projection.camera_fx * landscape_w / projection.camera_width;
    const float fy = projection.camera_fy * landscape_h / projection.camera_height;
    const float calibrated_cx = projection.camera_cx * landscape_w / projection.camera_width;
    const float calibrated_cy = projection.camera_cy * landscape_h / projection.camera_height;
    const float cx = landscape_w - 1.0f - calibrated_cx;
    const float cy = landscape_h - 1.0f - calibrated_cy;

    const float nx = vx / vz;
    const float ny = vy / vz;
    const float r2 = nx * nx + ny * ny;
    const float radial = 1.0f + projection.dist_k1 * r2 +
        projection.dist_k2 * r2 * r2 + projection.dist_k3 * r2 * r2 * r2;
    const float distorted_x = nx * radial - 2.0f * projection.dist_p1 * nx * ny -
        projection.dist_p2 * (r2 + 2.0f * nx * nx);
    const float distorted_y = ny * radial - projection.dist_p1 * (r2 + 2.0f * ny * ny) -
        2.0f * projection.dist_p2 * nx * ny;
    const float u_land = fx * distorted_x + cx;
    const float v_land = fy * distorted_y + cy;

    if (width > height) {
        *px = static_cast<int>(std::round(u_land));
        *py = static_cast<int>(std::round(v_land));
    } else {
        *px = static_cast<int>(std::round(v_land));
        *py = static_cast<int>(std::round(landscape_w - 1.0f - u_land));
    }
    return *px > -200 && *px < width + 200 && *py > -200 && *py < height + 200;
}
