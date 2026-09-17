#ifndef OVERLAY_RENDERER_H
#define OVERLAY_RENDERER_H

#include "overlay_state.h"
#include "model_output.h"
#include "projection.h"

#include <cstdint>
#include <memory>
#include <string>

/* 깜빡이 애니메이션은 켜진 순간을 0으로 하는 단계 수로 그린다. 단계 진행은
 * k230_overlayd가 시각 기준으로 계산하므로 재그리기 빈도에 영향받지 않는다. */
constexpr int kTurnSignalSteps = 25;

/* ARGB8888 CPU 버퍼. 렌더러는 DRM 헤더 없이 이 뷰만 본다. */
struct OverlayTarget {
    void *map = nullptr;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
};

struct TrafficSignalSprites;

class OverlayRenderer {
public:
    /* <dir>/traffic_*_retro-270x155-v3.png. 없으면 신호등 표시만 빠진다. */
    bool load_assets(const std::string &dir);

    void draw(const OverlayTarget &target, const ParsedModelOutput &output,
              const ProjectionState &projection,
              const OverlayHudState &hud = OverlayHudState{},
              bool rotate_landscape = false) const;

private:
    std::shared_ptr<const TrafficSignalSprites> sprites_;
};

#endif
