#include "overlay_renderer.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>

namespace {

cv::Scalar bgra(int b, int g, int r)
{
    return cv::Scalar(b, g, r, 255);
}

int line_width(int previous_radius)
{
    return std::max(1, previous_radius * 2 + 1);
}

void draw_triangle_marker(cv::Mat &img, int cx, int cy, int radius, const cv::Scalar &color)
{
    const cv::Point top(cx, cy - radius);
    const cv::Point left(cx - radius, cy + radius);
    const cv::Point right(cx + radius, cy + radius);
    constexpr int kOutlineRadius = 2;
    cv::line(img, top, left, color, line_width(kOutlineRadius), cv::LINE_8);
    cv::line(img, left, right, color, line_width(kOutlineRadius), cv::LINE_8);
    cv::line(img, right, top, color, line_width(kOutlineRadius), cv::LINE_8);
    cv::circle(img, cv::Point(cx, cy), std::max(2, radius / 4), color, cv::FILLED, cv::LINE_8);
}

void draw_points(cv::Mat &img,
                 const std::array<ModelPoint, kTrajectorySize> &points,
                 float z_offset, int previous_radius, const cv::Scalar &color,
                 const ProjectionState &projection)
{
    bool have_prev = false;
    int prev_x = 0;
    int prev_y = 0;
    for (const ModelPoint &point : points) {
        int px = 0;
        int py = 0;
        if (project_point(projection, point.x, point.y, point.z + z_offset,
                          img.cols, img.rows, &px, &py)) {
            if (have_prev)
                cv::line(img, cv::Point(prev_x, prev_y), cv::Point(px, py),
                         color, line_width(previous_radius), cv::LINE_8);
            prev_x = px;
            prev_y = py;
            have_prev = true;
        } else {
            have_prev = false;
        }
    }
}

} // namespace

void OverlayRenderer::draw(display_buffer *buffer, const ParsedModelOutput &output,
                           const ProjectionState &projection) const
{
    constexpr float kLeadProbabilityThreshold = 0.5f;
    constexpr int kLeadTimeIndex = 0;

    const int width = static_cast<int>(buffer->width);
    const int height = static_cast<int>(buffer->height);
    const int stride = static_cast<int>(buffer->stride);
    cv::Mat frame(height, width, CV_8UC4, buffer->map, static_cast<size_t>(stride));
    frame.setTo(cv::Scalar(0, 0, 0, 0));

    if (output.valid) {
        if (output.plan.valid) {
            draw_points(frame, output.plan.points,
                        kModelHeight, 4, bgra(0, 220, 255), projection);
        }

        for (const ParsedLaneLine &lane : output.lanes) {
            if (!lane.valid || lane.probability < 0.2f) continue;
            const int thickness = std::max(1, static_cast<int>(1 + lane.probability * 4.0f));
            draw_points(frame, lane.points,
                        0.0f, thickness, bgra(80, 255, 80), projection);
        }

        for (const ParsedRoadEdge &edge : output.road_edges) {
            if (!edge.valid) continue;
            draw_points(frame, edge.points,
                        0.0f, 2, bgra(80, 80, 255), projection);
        }

        ParsedLeadPoint lead;
        if (output.leads.primary(kLeadTimeIndex, kLeadProbabilityThreshold, &lead)) {
            int px = 0;
            int py = 0;
            if (project_point(projection, lead.x, lead.y, kModelHeight, width, height, &px, &py)) {
                const int radius = std::max(7, std::min(15, static_cast<int>(18.0f - lead.x * 0.08f)));
                draw_triangle_marker(frame, px, py, radius, bgra(255, 255, 255));
                cv::circle(frame, cv::Point(px, py), 3, bgra(0, 0, 255), cv::FILLED, cv::LINE_8);
            }
        }
    }
}
