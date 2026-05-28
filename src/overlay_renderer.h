#ifndef OVERLAY_RENDERER_H
#define OVERLAY_RENDERER_H

#include "display.h"
#include "model_output.h"
#include "projection.h"

class OverlayRenderer {
public:
    OverlayRenderer() = default;

    void draw(display_buffer *buffer, const ParsedModelOutput &output,
              const ProjectionState &projection) const;
};

#endif
