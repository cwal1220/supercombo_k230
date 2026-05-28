#ifndef OVERLAY_RENDERER_H
#define OVERLAY_RENDERER_H

#include "app_config.h"
#include "display.h"
#include "model_output.h"
#include "projection.h"

class OverlayRenderer {
public:
    explicit OverlayRenderer(const AppConfig &config);

    void draw(display_buffer *buffer, const ParsedModelOutput &output,
              const ProjectionState &projection) const;

private:
    bool draw_lead_ = true;
    float lead_prob_threshold_ = 0.5f;
    int lead_time_idx_ = 0;
};

#endif
