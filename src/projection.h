#ifndef PROJECTION_H
#define PROJECTION_H

#include "app_config.h"

struct ProjectionState {
    ProjectionMode mode = ProjectionMode::Legacy;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float view_from_calib[9] = {
        0.0f, -1.0f, 0.0f,
        0.0f, 0.0f, -1.0f,
        1.0f, 0.0f, 0.0f,
    };
};

ProjectionState make_projection_state(ProjectionMode mode, float roll, float pitch, float yaw);
bool project_point(const ProjectionState &projection, float x_forward, float y_left, float z_up,
                   int width, int height, int *px, int *py);

#endif
