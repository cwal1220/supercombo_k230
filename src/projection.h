#ifndef PROJECTION_H
#define PROJECTION_H

#include "app_config.h"

struct ProjectionState {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float camera_fx = kK230CameraFx;
    float camera_fy = kK230CameraFy;
    float camera_cx = kK230CameraCx;
    float camera_cy = kK230CameraCy;
    float camera_width = static_cast<float>(kDefaultSensorWidth);
    float camera_height = static_cast<float>(kDefaultSensorHeight);
    float dist_k1 = kK230CameraK1;
    float dist_k2 = kK230CameraK2;
    float dist_p1 = kK230CameraP1;
    float dist_p2 = kK230CameraP2;
    float dist_k3 = kK230CameraK3;
    float view_from_calib[9] = {
        0.0f, -1.0f, 0.0f,
        0.0f, 0.0f, -1.0f,
        1.0f, 0.0f, 0.0f,
    };
};

ProjectionState make_projection_state(float roll, float pitch, float yaw);
ProjectionState make_projection_state(float roll, float pitch, float yaw,
                                      const AppConfig &config);
bool project_point(const ProjectionState &projection, float x_forward, float y_left, float z_up,
                   int width, int height, int *px, int *py);

#endif
