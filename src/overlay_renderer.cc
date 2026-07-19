#include "overlay_renderer.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstdio>
#include <string>

namespace {

cv::Scalar bgra(int b, int g, int r)
{
    return cv::Scalar(b, g, r, 255);
}

uint32_t argb(uint8_t a, uint8_t r, uint8_t g, uint8_t b)
{
    return (static_cast<uint32_t>(a) << 24) |
           (static_cast<uint32_t>(r) << 16) |
           (static_cast<uint32_t>(g) << 8) |
           static_cast<uint32_t>(b);
}

template <typename... Args>
std::string format_text(const char *format, Args... args)
{
    char text[128];
    std::snprintf(text, sizeof(text), format, args...);
    return text;
}

class BitmapHud {
public:
    BitmapHud(cv::Mat &frame, int logical_width, int logical_height, bool rotate_landscape)
        : frame_(frame),
          logical_width_(logical_width),
          logical_height_(logical_height),
          rotate_landscape_(rotate_landscape)
    {
    }

    void fill_rect(int x, int y, int w, int h, uint32_t color)
    {
        if (w <= 0 || h <= 0) return;
        const int x0 = std::max(0, x);
        const int y0 = std::max(0, y);
        const int x1 = std::min(logical_width_, x + w);
        const int y1 = std::min(logical_height_, y + h);
        if (x0 >= x1 || y0 >= y1) return;

        const int native_x0 = rotate_landscape_ ? logical_height_ - y1 : x0;
        const int native_y0 = rotate_landscape_ ? x0 : y0;
        const int native_x1 = rotate_landscape_ ? logical_height_ - y0 : x1;
        const int native_y1 = rotate_landscape_ ? x1 : y1;
        for (int row = native_y0; row < native_y1; ++row) {
            uint32_t *pixels = frame_.ptr<uint32_t>(row);
            std::fill(pixels + native_x0, pixels + native_x1, color);
        }
    }

    void text_left(int x, int y, const std::string &raw_text, int scale,
                   uint32_t color, int max_width = 0)
    {
        if (scale <= 0) return;
        const std::string text = clip_text(raw_text, scale, max_width);
        int cursor_x = x;
        for (char raw : text) {
            char c = raw;
            if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
            const std::array<uint8_t, 7> rows = glyph_rows(c);
            for (int row = 0; row < 7; ++row) {
                for (int col = 0; col < 5; ++col) {
                    if (rows[row] & (1U << (4 - col)))
                        fill_rect(cursor_x + col * scale, y + row * scale,
                                  scale, scale, color);
                }
            }
            cursor_x += 6 * scale;
        }
    }

    void hud_text_left(int x, int y, const std::string &text, int scale,
                       uint32_t color, int max_width = 0)
    {
        text_left(x + 2, y + 2, text, scale, argb(185, 0, 0, 0), max_width);
        text_left(x, y, text, scale, color, max_width);
    }

    void hud_text_center(int center_x, int y, const std::string &raw_text, int scale,
                         uint32_t color, int max_width = 0)
    {
        const std::string text = clip_text(raw_text, scale, max_width);
        const int x = center_x - text_width(text, scale) / 2;
        text_left(x + 2, y + 2, text, scale, argb(185, 0, 0, 0));
        text_left(x, y, text, scale, color);
    }

    void box(int x, int y, int w, int h, uint32_t accent, bool accent_right = false)
    {
        fill_rect(x, y, w, h, argb(74, 0, 0, 0));
        fill_rect(x, y, w, 1, argb(72, 255, 255, 255));
        fill_rect(x, y + h - 1, w, 1, argb(36, 255, 255, 255));
        fill_rect(accent_right ? x + w - 3 : x, y, 3, h, accent);
    }

private:
    static int text_width(const std::string &text, int scale)
    {
        return text.empty() ? 0 : static_cast<int>(text.size()) * 6 * scale - scale;
    }

    static std::string clip_text(const std::string &text, int scale, int max_width)
    {
        if (max_width <= 0 || text_width(text, scale) <= max_width) return text;
        std::string clipped = text;
        while (!clipped.empty() && text_width(clipped, scale) > max_width)
            clipped.pop_back();
        return clipped;
    }

    static std::array<uint8_t, 7> glyph_rows(char c)
    {
        switch (c) {
        case '0': return {0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e};
        case '1': return {0x04, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e};
        case '2': return {0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f};
        case '3': return {0x1e, 0x01, 0x01, 0x0e, 0x01, 0x01, 0x1e};
        case '4': return {0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02};
        case '5': return {0x1f, 0x10, 0x1e, 0x01, 0x01, 0x11, 0x0e};
        case '6': return {0x06, 0x08, 0x10, 0x1e, 0x11, 0x11, 0x0e};
        case '7': return {0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08};
        case '8': return {0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e};
        case '9': return {0x0e, 0x11, 0x11, 0x0f, 0x01, 0x02, 0x0c};
        case 'A': return {0x0e, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11};
        case 'B': return {0x1e, 0x11, 0x11, 0x1e, 0x11, 0x11, 0x1e};
        case 'C': return {0x0e, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0e};
        case 'D': return {0x1e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1e};
        case 'E': return {0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x1f};
        case 'F': return {0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10};
        case 'G': return {0x0e, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0f};
        case 'H': return {0x11, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11};
        case 'I': return {0x0e, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0e};
        case 'J': return {0x07, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0c};
        case 'K': return {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11};
        case 'L': return {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f};
        case 'M': return {0x11, 0x1b, 0x15, 0x15, 0x11, 0x11, 0x11};
        case 'N': return {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11};
        case 'O': return {0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e};
        case 'P': return {0x1e, 0x11, 0x11, 0x1e, 0x10, 0x10, 0x10};
        case 'Q': return {0x0e, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0d};
        case 'R': return {0x1e, 0x11, 0x11, 0x1e, 0x14, 0x12, 0x11};
        case 'S': return {0x0f, 0x10, 0x10, 0x0e, 0x01, 0x01, 0x1e};
        case 'T': return {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04};
        case 'U': return {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e};
        case 'V': return {0x11, 0x11, 0x11, 0x11, 0x11, 0x0a, 0x04};
        case 'W': return {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0a};
        case 'X': return {0x11, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x11};
        case 'Y': return {0x11, 0x11, 0x0a, 0x04, 0x04, 0x04, 0x04};
        case 'Z': return {0x1f, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1f};
        case '-': return {0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00};
        case '.': return {0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x0c};
        case ':': return {0x00, 0x04, 0x04, 0x00, 0x04, 0x04, 0x00};
        case '/': return {0x01, 0x02, 0x02, 0x04, 0x08, 0x08, 0x10};
        case '%': return {0x19, 0x1a, 0x02, 0x04, 0x08, 0x0b, 0x13};
        case '<': return {0x02, 0x04, 0x08, 0x10, 0x08, 0x04, 0x02};
        case '>': return {0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08};
        case '!': return {0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x04};
        case '?': return {0x0e, 0x11, 0x01, 0x02, 0x04, 0x00, 0x04};
        case '+': return {0x00, 0x04, 0x04, 0x1f, 0x04, 0x04, 0x00};
        case '=': return {0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00};
        case '_': return {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f};
        default: return {0, 0, 0, 0, 0, 0, 0};
        }
    }

    cv::Mat &frame_;
    int logical_width_ = 0;
    int logical_height_ = 0;
    bool rotate_landscape_ = false;
};

int line_width(int previous_radius)
{
    return std::max(1, previous_radius * 2 + 1);
}

void rotate_model_point_180(int width, int height, int *x, int *y)
{
    *x = width - 1 - *x;
    *y = height - 1 - *y;
}

cv::Point display_point(int x, int y, int logical_height, bool rotate_landscape)
{
    return rotate_landscape ? cv::Point(logical_height - 1 - y, x) : cv::Point(x, y);
}

void draw_triangle_marker_180(cv::Mat &img, int cx, int cy, int radius,
                              const cv::Scalar &color, int logical_height,
                              bool rotate_landscape)
{
    const cv::Point tip = display_point(cx, cy + radius, logical_height, rotate_landscape);
    const cv::Point left = display_point(cx - radius, cy - radius,
                                         logical_height, rotate_landscape);
    const cv::Point right = display_point(cx + radius, cy - radius,
                                          logical_height, rotate_landscape);
    const cv::Point center = display_point(cx, cy, logical_height, rotate_landscape);
    constexpr int kOutlineRadius = 2;
    cv::line(img, tip, left, color, line_width(kOutlineRadius), cv::LINE_8);
    cv::line(img, left, right, color, line_width(kOutlineRadius), cv::LINE_8);
    cv::line(img, right, tip, color, line_width(kOutlineRadius), cv::LINE_8);
    cv::circle(img, center, std::max(2, radius / 4), color, cv::FILLED, cv::LINE_8);
}

void draw_hud(cv::Mat &frame, const OverlayHudState &hud,
              const ParsedModelOutput &output, int logical_width, int logical_height,
              bool rotate_landscape)
{
    const int width = logical_width;
    const int height = logical_height;
    BitmapHud ui(frame, logical_width, logical_height, rotate_landscape);
    const uint32_t white = argb(230, 255, 255, 255);
    const uint32_t dim = argb(170, 210, 220, 230);
    const uint32_t green = argb(230, 80, 230, 95);
    const uint32_t blue = argb(230, 90, 170, 255);
    const uint32_t yellow = argb(230, 255, 220, 60);
    const uint32_t orange = argb(235, 255, 150, 50);
    const uint32_t red = argb(235, 255, 70, 70);

    const uint32_t status = hud.steering_fault || hud.panda_faults != 0 ? red :
                            (!hud.services_healthy ? orange :
                             (hud.controller_active ? green :
                              (hud.controller_enabled ? blue : argb(220, 110, 120, 130))));
    ui.fill_rect(0, 0, width, 6, status);

    constexpr int box_w = 236;
    constexpr int left_box_x = 8;
    const int right_box_x = width - box_w - 8;
    constexpr int left_x = left_box_x + 10;
    const int right_x = right_box_x + 10;
    constexpr int col_w = box_w - 20;

    ui.hud_text_center(width / 2, 12,
                       format_text("%.0F", std::max(0.0f, hud.speed_kph)),
                       8, white, 220);
    ui.hud_text_center(width / 2, 80, "KPH", 2, dim, 140);

    const uint32_t control_color = hud.steering_fault ? red :
                                   (hud.controller_active ? green :
                                    (hud.controller_engaged ? yellow : dim));
    const char *op_status = hud.controller_active ? "OP ACT" :
                            (hud.controller_engaged ? "OP EN" :
                             (hud.controller_enabled ? "OP RDY" : "OP OFF"));
    const uint32_t panda_color = !hud.panda_connected || !hud.panda_healthy ? orange :
                                 (hud.panda_faults != 0 ? red : green);

    ui.box(left_box_x, 10, box_w, 72, status);
    ui.hud_text_left(left_x, 16, "OPENPILOT", 1, dim, col_w);
    ui.hud_text_left(left_x, 34, op_status, 3, control_color, col_w);
    ui.hud_text_left(left_x, 62,
                     format_text("PANDA %s  CAR %s",
                                 hud.panda_connected && hud.panda_healthy ? "OK" : "--",
                                 hud.vehicle_fresh ? "OK" : "--"),
                     1, panda_color, col_w);

    const uint32_t cpu_color = hud.cpu_percent >= 90.0f || hud.cpu_temp_c >= 85.0f ? red :
                               (hud.cpu_percent >= 75.0f || hud.cpu_temp_c >= 75.0f ? orange :
                                (hud.cpu_percent >= 60.0f || hud.cpu_temp_c >= 65.0f ? yellow : dim));
    ui.box(right_box_x, 10, box_w, 72, cpu_color, true);
    ui.hud_text_left(right_x, 16,
                     format_text("CPU %.0F%%  AI %.1F", hud.cpu_percent, hud.model_fps),
                     2, cpu_color, col_w);
    ui.hud_text_left(right_x, 40,
                     format_text("CAM %.1F  HUD %.1F", hud.preview_fps, hud.overlay_fps),
                     2, dim, col_w);
    ui.hud_text_left(right_x, 64,
                     format_text("MODEL %.0FMS", hud.model_execution_ms),
                     1, dim, col_w);

    constexpr int control_y = 420;
    ui.box(left_box_x, control_y, width - 16, 52, status);
    ui.hud_text_left(left_x, control_y + 8,
                     format_text("ANGLE %.1F  TORQUE %d/%d  DRIVER %d",
                                 hud.steering_angle_deg, hud.desired_torque,
                                 hud.apply_torque, hud.driver_torque),
                     2, control_color, width - 36);
    ui.hud_text_left(left_x, control_y + 32,
                     format_text("MODEL %s  CONTROL %s  TX %s",
                                 output.valid ? "OK" : "--",
                                 hud.panda_controls_allowed ? "OK" : "--",
                                 hud.panda_tx_enabled ? "ON" : "SHADOW"),
                     1, hud.services_healthy ? dim : orange, width - 36);

    std::string alert;
    uint32_t alert_color = orange;
    if (hud.steering_fault) {
        alert = "STEERING FAULT";
        alert_color = red;
    } else if (hud.panda_faults != 0) {
        alert = "PANDA FAULT";
        alert_color = red;
    } else if (!hud.services_healthy) {
        alert = "WAITING FOR SERVICES";
    }
    if (!alert.empty()) {
        constexpr int alert_w = 520;
        const int alert_x = (width - alert_w) / 2;
        const int alert_y = height - 58;
        ui.box(alert_x, alert_y, alert_w, 50, alert_color);
        ui.hud_text_center(width / 2, alert_y + 8, alert, 3,
                           alert_color, alert_w - 18);
        ui.hud_text_center(width / 2, alert_y + 36,
                           format_text("M%s CTL%s PND%s",
                                       output.valid ? "OK" : "--",
                                       hud.vehicle_fresh ? "OK" : "--",
                                       hud.panda_connected ? "OK" : "--"),
                           1, white, alert_w - 18);
    }
}

void draw_points(cv::Mat &img,
                 const std::array<ModelPoint, kTrajectorySize> &points,
                 float z_offset, int previous_radius, const cv::Scalar &color,
                 const ProjectionState &projection, int logical_width, int logical_height,
                 bool rotate_landscape)
{
    bool have_prev = false;
    int prev_x = 0;
    int prev_y = 0;
    for (const ModelPoint &point : points) {
        int px = 0;
        int py = 0;
        if (project_point(projection, point.x, point.y, point.z + z_offset,
                          logical_width, logical_height, &px, &py)) {
            rotate_model_point_180(logical_width, logical_height, &px, &py);
            const cv::Point current = display_point(px, py, logical_height, rotate_landscape);
            if (have_prev)
                cv::line(img, cv::Point(prev_x, prev_y), current,
                         color, line_width(previous_radius), cv::LINE_8);
            prev_x = current.x;
            prev_y = current.y;
            have_prev = true;
        } else {
            have_prev = false;
        }
    }
}

} // namespace

void OverlayRenderer::draw(display_buffer *buffer, const ParsedModelOutput &output,
                           const ProjectionState &projection,
                           const OverlayHudState &hud,
                           bool rotate_landscape) const
{
    constexpr float kLeadProbabilityThreshold = 0.5f;
    constexpr int kLeadTimeIndex = 0;

    const int width = static_cast<int>(buffer->width);
    const int height = static_cast<int>(buffer->height);
    const int stride = static_cast<int>(buffer->stride);
    const int logical_width = rotate_landscape ? height : width;
    const int logical_height = rotate_landscape ? width : height;
    cv::Mat frame(height, width, CV_8UC4, buffer->map, static_cast<size_t>(stride));
    frame.setTo(cv::Scalar(0, 0, 0, 0));

    if (output.valid) {
        if (output.plan.valid) {
            const cv::Scalar path_color = hud.controller_active
                ? bgra(70, 230, 90)
                : (hud.controller_engaged ? bgra(40, 210, 255) : bgra(0, 220, 255));
            draw_points(frame, output.plan.points,
                        kModelHeight, 4, path_color, projection,
                        logical_width, logical_height, rotate_landscape);
        }

        for (const ParsedLaneLine &lane : output.lanes) {
            if (!lane.valid || lane.probability < 0.2f) continue;
            const int thickness = std::max(1, static_cast<int>(1 + lane.probability * 4.0f));
            draw_points(frame, lane.points,
                        0.0f, thickness, bgra(80, 255, 80), projection,
                        logical_width, logical_height, rotate_landscape);
        }

        for (const ParsedRoadEdge &edge : output.road_edges) {
            if (!edge.valid) continue;
            draw_points(frame, edge.points,
                        0.0f, 2, bgra(80, 80, 255), projection,
                        logical_width, logical_height, rotate_landscape);
        }

        ParsedLeadPoint lead;
        if (output.leads.primary(kLeadTimeIndex, kLeadProbabilityThreshold, &lead)) {
            int px = 0;
            int py = 0;
            if (project_point(projection, lead.x, lead.y, kModelHeight,
                              logical_width, logical_height, &px, &py)) {
                rotate_model_point_180(logical_width, logical_height, &px, &py);
                const int radius = std::max(7, std::min(15, static_cast<int>(18.0f - lead.x * 0.08f)));
                draw_triangle_marker_180(frame, px, py, radius, bgra(255, 255, 255),
                                         logical_height, rotate_landscape);
                cv::circle(frame, display_point(px, py, logical_height, rotate_landscape),
                           3, bgra(0, 0, 255), cv::FILLED, cv::LINE_8);
            }
        }
    }

    draw_hud(frame, hud, output, logical_width, logical_height, rotate_landscape);
}
