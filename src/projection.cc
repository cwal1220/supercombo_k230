#include "projection.h"

#include <cmath>

namespace {

void matmul3(const float *a, const float *b, float *out)
{
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < 3; ++k)
                sum += a[r * 3 + k] * b[k * 3 + c];
            out[r * 3 + c] = sum;
        }
    }
}

void rotation_from_rpy(float roll, float pitch, float yaw, float *rot)
{
    const float cr = std::cos(roll);
    const float sr = std::sin(roll);
    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);
    const float cy = std::cos(yaw);
    const float sy = std::sin(yaw);

    const float rx[9] = {
        1.0f, 0.0f, 0.0f,
        0.0f, cr, -sr,
        0.0f, sr, cr,
    };
    const float ry[9] = {
        cp, 0.0f, sp,
        0.0f, 1.0f, 0.0f,
        -sp, 0.0f, cp,
    };
    const float rz[9] = {
        cy, -sy, 0.0f,
        sy, cy, 0.0f,
        0.0f, 0.0f, 1.0f,
    };

    float tmp[9];
    matmul3(ry, rx, tmp);
    matmul3(rz, tmp, rot);
}

} // namespace

ProjectionState make_projection_state(ProjectionMode mode, float roll, float pitch, float yaw)
{
    ProjectionState state;
    state.mode = mode;
    state.roll = roll;
    state.pitch = pitch;
    state.yaw = yaw;

    float rot[9];
    rotation_from_rpy(roll, pitch, yaw, rot);

    float device_from_calib[9];
    if (mode == ProjectionMode::Legacy) {
        for (int row = 0; row < 3; ++row) {
            device_from_calib[row * 3 + 0] = rot[row * 3 + 0];
            device_from_calib[row * 3 + 1] = -rot[row * 3 + 1];
            device_from_calib[row * 3 + 2] = -rot[row * 3 + 2];
        }
    } else {
        for (int i = 0; i < 9; ++i)
            device_from_calib[i] = rot[i];
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
    const float fx = 910.0f * landscape_w / 1164.0f;
    const float fy = 910.0f * landscape_h / 874.0f;
    const float cx = landscape_w * 0.5f;
    const float cy = landscape_h * 0.50f;

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
