#include "overlay_renderer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace {

struct Bgra {
    uint8_t b;
    uint8_t g;
    uint8_t r;
    uint8_t a;
};

Bgra bgra(uint8_t b, uint8_t g, uint8_t r)
{
    return Bgra{b, g, r, 255};
}

void put_pixel(uint8_t *img, int stride, int width, int height, int x, int y, Bgra color)
{
    if (x < 0 || x >= width || y < 0 || y >= height) return;
    uint8_t *p = img + y * stride + x * 4;
    p[0] = color.b;
    p[1] = color.g;
    p[2] = color.r;
    p[3] = color.a;
}

void draw_disc(uint8_t *img, int stride, int width, int height, int cx, int cy, int radius, Bgra color)
{
    for (int y = cy - radius; y <= cy + radius; ++y) {
        for (int x = cx - radius; x <= cx + radius; ++x) {
            const int dx = x - cx;
            const int dy = y - cy;
            if (dx * dx + dy * dy <= radius * radius)
                put_pixel(img, stride, width, height, x, y, color);
        }
    }
}

void draw_line(uint8_t *img, int stride, int width, int height,
               int x0, int y0, int x1, int y1, int thickness, Bgra color)
{
    const int dx = std::abs(x1 - x0);
    const int sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0);
    const int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        draw_disc(img, stride, width, height, x0, y0, thickness, color);
        if (x0 == x1 && y0 == y1) break;
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void draw_triangle_marker(uint8_t *img, int stride, int width, int height,
                          int cx, int cy, int radius, Bgra color)
{
    const int x0 = cx;
    const int y0 = cy - radius;
    const int x1 = cx - radius;
    const int y1 = cy + radius;
    const int x2 = cx + radius;
    const int y2 = cy + radius;
    draw_line(img, stride, width, height, x0, y0, x1, y1, 2, color);
    draw_line(img, stride, width, height, x1, y1, x2, y2, 2, color);
    draw_line(img, stride, width, height, x2, y2, x0, y0, 2, color);
    draw_disc(img, stride, width, height, cx, cy, std::max(2, radius / 4), color);
}

void draw_points(uint8_t *img, int stride, int width, int height,
                 const std::array<ModelPoint, kTrajectorySize> &points,
                 float z_offset, int thickness, Bgra color,
                 const ProjectionState &projection)
{
    bool have_prev = false;
    int prev_x = 0;
    int prev_y = 0;
    for (const ModelPoint &point : points) {
        int px = 0;
        int py = 0;
        if (project_point(projection, point.x, point.y, point.z + z_offset, width, height, &px, &py)) {
            if (have_prev)
                draw_line(img, stride, width, height, prev_x, prev_y, px, py, thickness, color);
            prev_x = px;
            prev_y = py;
            have_prev = true;
        } else {
            have_prev = false;
        }
    }
}

} // namespace

OverlayRenderer::OverlayRenderer(const AppConfig &config)
    : draw_lead_(config.draw_lead),
      lead_prob_threshold_(config.lead_prob_threshold),
      lead_time_idx_(config.lead_time_idx)
{
}

void OverlayRenderer::draw(display_buffer *buffer, const ParsedModelOutput &output,
                           const ProjectionState &projection) const
{
    const int width = static_cast<int>(buffer->width);
    const int height = static_cast<int>(buffer->height);
    const int stride = static_cast<int>(buffer->stride);
    uint8_t *frame = static_cast<uint8_t *>(buffer->map);
    std::memset(frame, 0, buffer->size);

    if (output.valid) {
        if (output.plan.valid) {
            draw_points(frame, stride, width, height, output.plan.points,
                        kModelHeight, 4, bgra(0, 220, 255), projection);
        }

        for (const ParsedLaneLine &lane : output.lanes) {
            if (!lane.valid || lane.probability < 0.2f) continue;
            const int thickness = std::max(1, static_cast<int>(1 + lane.probability * 4.0f));
            draw_points(frame, stride, width, height, lane.points,
                        0.0f, thickness, bgra(80, 255, 80), projection);
        }

        for (const ParsedRoadEdge &edge : output.road_edges) {
            if (!edge.valid) continue;
            draw_points(frame, stride, width, height, edge.points,
                        0.0f, 2, bgra(80, 80, 255), projection);
        }

        ParsedLeadPoint lead;
        if (draw_lead_ && output.leads.primary(lead_time_idx_, lead_prob_threshold_, &lead)) {
            int px = 0;
            int py = 0;
            if (project_point(projection, lead.x, lead.y, kModelHeight, width, height, &px, &py)) {
                const int radius = std::max(7, std::min(15, static_cast<int>(18.0f - lead.x * 0.08f)));
                draw_triangle_marker(frame, stride, width, height, px, py, radius, bgra(255, 255, 255));
                draw_disc(frame, stride, width, height, px, py, 3, bgra(0, 0, 255));
            }
        }
    }
}
