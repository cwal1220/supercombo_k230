#include "overlay_renderer.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace {

cv::Scalar bgra(int b, int g, int r, int a = 255)
{
    return cv::Scalar(b, g, r, a);
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

void rotate_model_point_180(int width, int height, int *x, int *y)
{
    *x = width - 1 - *x;
    *y = height - 1 - *y;
}

cv::Point display_point(int x, int y, int logical_height, bool rotate_landscape)
{
    return rotate_landscape ? cv::Point(logical_height - 1 - y, x) : cv::Point(x, y);
}

void draw_turn_chevron(cv::Mat &frame, int inner_x, int center_y, bool points_left,
                       int alpha, int logical_height, bool rotate_landscape)
{
    constexpr int kChevronWidth = 24;
    constexpr int kChevronHalfHeight = 34;
    const int direction = points_left ? -1 : 1;
    const int shoulder_x = inner_x + direction * (kChevronWidth / 2);
    const int tip_x = inner_x + direction * kChevronWidth;

    std::array<cv::Point, 6> vertices = {
        display_point(inner_x, center_y - kChevronHalfHeight,
                      logical_height, rotate_landscape),
        display_point(shoulder_x, center_y - kChevronHalfHeight,
                      logical_height, rotate_landscape),
        display_point(tip_x, center_y, logical_height, rotate_landscape),
        display_point(shoulder_x, center_y + kChevronHalfHeight,
                      logical_height, rotate_landscape),
        display_point(inner_x, center_y + kChevronHalfHeight,
                      logical_height, rotate_landscape),
        display_point(shoulder_x, center_y, logical_height, rotate_landscape),
    };
    const cv::Point *points = vertices.data();
    const int point_count = static_cast<int>(vertices.size());
    cv::fillPoly(frame, &points, &point_count, 1, bgra(70, 230, 255, alpha),
                 cv::LINE_8);
}

void draw_turn_signals(cv::Mat &frame, const OverlayHudState &hud,
                       int blinking_rate, int logical_width, int logical_height,
                       bool rotate_landscape)
{
    if (!hud.left_blinker && !hud.right_blinker) return;

    constexpr int kCenterY = 130;
    constexpr int kInnerOffset = 74;
    constexpr int kChevronStep = 28;
    const int center_x = logical_width / 2;

    auto draw_side = [&](bool active, bool points_left) {
        if (!active) return;
        const int direction = points_left ? -1 : 1;
        const int first_inner_x = center_x + direction * kInnerOffset;
        if (blinking_rate <= 120 && blinking_rate >= 50) {
            draw_turn_chevron(frame, first_inner_x, kCenterY, points_left, 70,
                              logical_height, rotate_landscape);
        }
        if (blinking_rate <= 100 && blinking_rate >= 50) {
            draw_turn_chevron(frame, first_inner_x + direction * kChevronStep,
                              kCenterY, points_left, 140,
                              logical_height, rotate_landscape);
        }
        if (blinking_rate <= 80 && blinking_rate >= 50) {
            draw_turn_chevron(frame, first_inner_x + direction * (2 * kChevronStep),
                              kCenterY, points_left, 210,
                              logical_height, rotate_landscape);
        }
    };

    draw_side(hud.left_blinker, true);
    draw_side(hud.right_blinker, false);
}

void draw_lead_chevron_180(cv::Mat &img, int cx, int cy, int size,
                           float distance_m, float relative_speed_mps,
                           float probability, int logical_width, int logical_height,
                           bool rotate_landscape)
{
    const int outer_half_width = size * 5 / 4;
    cx = std::clamp(cx, outer_half_width + 4, logical_width - outer_half_width - 4);
    cy = std::clamp(cy, size + 4, logical_height - size - 4);

    const float distance_risk = std::clamp(1.0f - distance_m / 40.0f, 0.0f, 1.0f);
    const float closing_risk = std::clamp(-relative_speed_mps / 10.0f, 0.0f, 1.0f);
    const float risk = std::clamp(distance_risk + closing_risk, 0.0f, 1.0f);
    const int confidence_alpha = static_cast<int>(
        170.0f + 85.0f * std::clamp(probability, 0.0f, 1.0f));
    const int inner_green = static_cast<int>(165.0f - 125.0f * risk);

    auto fill_triangle = [&](int center_y, int half_width, int half_height,
                             const cv::Scalar &color) {
        std::array<cv::Point, 3> vertices = {
            display_point(cx, center_y - half_height, logical_height, rotate_landscape),
            display_point(cx - half_width, center_y + half_height,
                          logical_height, rotate_landscape),
            display_point(cx + half_width, center_y + half_height,
                          logical_height, rotate_landscape),
        };
        cv::fillConvexPoly(img, vertices.data(), static_cast<int>(vertices.size()),
                           color, cv::LINE_8);
    };

    fill_triangle(cy + 2, outer_half_width + 3, size + 3, bgra(0, 0, 0, 130));
    fill_triangle(cy, outer_half_width, size, bgra(35, 220, 255, confidence_alpha));
    fill_triangle(cy - 1, std::max(4, outer_half_width - 4), std::max(4, size - 4),
                  bgra(35, inner_green, 255, confidence_alpha));
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
    ui.box(left_box_x, 90, box_w, 66, control_color);
    ui.hud_text_left(left_x, 96,
                     format_text("ANGLE %.1F", hud.steering_angle_deg),
                     2, control_color, col_w);
    ui.hud_text_left(left_x, 116,
                     format_text("TQ D %d A %d", hud.desired_torque, hud.apply_torque),
                     2, control_color, col_w);
    ui.hud_text_left(left_x, 136,
                     format_text("DRIVER %d", hud.driver_torque),
                     2, hud.services_healthy ? dim : orange, col_w);

    const uint32_t cpu_color = hud.cpu_percent >= 90.0f || hud.cpu_temp_c >= 85.0f ? red :
                               (hud.cpu_percent >= 75.0f || hud.cpu_temp_c >= 75.0f ? orange :
                                (hud.cpu_percent >= 60.0f || hud.cpu_temp_c >= 65.0f ? yellow : dim));
    const char *calibration_status = !hud.calibration_available ? "--" :
                                     (hud.calibration_status == 1 ? "OK" :
                                      (hud.calibration_status == 2 ? "BAD" : "WAIT"));
    const uint32_t calibration_color = !hud.calibration_available ? dim :
                                       (hud.calibration_status == 1 ? green :
                                        (hud.calibration_status == 2 ? red : yellow));
    ui.box(right_box_x, 10, box_w, 72, cpu_color, true);
    ui.hud_text_left(right_x, 16,
                     format_text("CPU %.0F%% TEMP %.0FC", hud.cpu_percent, hud.cpu_temp_c),
                     2, cpu_color, col_w);
    ui.hud_text_left(right_x, 42,
                     format_text("AI %.1F FPS CAM %.1F FPS",
                                 hud.model_fps, hud.preview_fps),
                     1, dim, col_w);
    ui.hud_text_left(right_x, 58,
                     format_text("HUD %.1F FPS MEM %.0F%%",
                                 hud.overlay_fps, hud.memory_percent),
                     1, dim, col_w);

    ui.box(right_box_x, 90, box_w, 66, calibration_color, true);
    ui.hud_text_left(right_x, 96,
                     format_text("CAL %s B %d", calibration_status,
                                 std::max(0, hud.calibration_valid_blocks)),
                     2, calibration_color, col_w);
    ui.hud_text_left(right_x, 116,
                     format_text("R %.2F P %.2F", hud.calibration_roll_deg,
                                 hud.calibration_pitch_deg),
                     2, calibration_color, col_w);
    ui.hud_text_left(right_x, 136,
                     format_text("Y %.2F DEG", hud.calibration_yaw_deg),
                     2, calibration_color, col_w);

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

bool append_projected_point(std::vector<cv::Point> *vertices,
                            const ModelPoint &point, float y_offset, float z_offset,
                            const ProjectionState &projection,
                            int logical_width, int logical_height,
                            bool rotate_landscape)
{
    int px = 0;
    int py = 0;
    if (!project_point(projection, point.x, point.y + y_offset, point.z + z_offset,
                       logical_width, logical_height, &px, &py)) {
        return false;
    }

    rotate_model_point_180(logical_width, logical_height, &px, &py);
    vertices->push_back(display_point(px, py, logical_height, rotate_landscape));
    return true;
}

void draw_model_ribbon(cv::Mat &img,
                       const std::array<ModelPoint, kTrajectorySize> &points,
                       float half_width, float z_offset, float max_distance,
                       const cv::Scalar &color, const ProjectionState &projection,
                       int logical_width, int logical_height, bool rotate_landscape)
{
    int max_idx = 0;
    while (max_idx + 1 < kTrajectorySize && points[max_idx + 1].x <= max_distance)
        ++max_idx;

    std::vector<cv::Point> vertices;
    vertices.reserve(static_cast<size_t>(max_idx + 1) * 2);
    for (int i = 0; i <= max_idx; ++i)
        append_projected_point(&vertices, points[i], -half_width, z_offset, projection,
                               logical_width, logical_height, rotate_landscape);
    for (int i = max_idx; i >= 0; --i)
        append_projected_point(&vertices, points[i], half_width, z_offset, projection,
                               logical_width, logical_height, rotate_landscape);

    if (vertices.size() >= 3) {
        const cv::Point *polygon[] = {vertices.data()};
        const int count[] = {static_cast<int>(vertices.size())};
        cv::fillPoly(img, polygon, count, 1, color, cv::LINE_8);
    }
}

cv::Scalar openpilot_path_color(const OverlayHudState &hud)
{
    if (!hud.controller_engaged)
        return bgra(255, 255, 255, 150);

    const float output_scale = std::clamp(std::abs(hud.normalized_output) * 0.9f,
                                          0.0f, 1.0f);
    const int red = static_cast<int>(output_scale * 255.0f);
    const int green = static_cast<int>((1.0f - output_scale) * 255.0f);
    return bgra(0, green, red, 160);
}

cv::Scalar openpilot_lane_color(float probability)
{
    float red = 255.0f;
    float green = 255.0f;
    if (probability > 0.4f)
        red = (1.0f - (probability - 0.4f) * 2.5f) * 255.0f;
    else
        green = (1.0f - (0.4f - probability) * 2.5f) * 255.0f;

    return bgra(0,
                static_cast<int>(std::clamp(green, 0.0f, 255.0f)),
                static_cast<int>(std::clamp(red, 0.0f, 255.0f)),
                static_cast<int>(std::clamp(probability, 0.0f, 1.0f) * 230.0f));
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
        constexpr float kMinDrawDistance = 10.0f;
        constexpr float kMaxDrawDistance = 100.0f;
        const float max_distance = output.plan.valid
            ? std::clamp(output.plan.points.back().x, kMinDrawDistance, kMaxDrawDistance)
            : kMaxDrawDistance;

        if (output.plan.valid) {
            draw_model_ribbon(frame, output.plan.points,
                              0.9f, kModelHeight, max_distance,
                              openpilot_path_color(hud), projection,
                              logical_width, logical_height, rotate_landscape);
        }

        for (const ParsedLaneLine &lane : output.lanes) {
            if (!lane.valid || lane.probability < 0.05f) continue;
            draw_model_ribbon(frame, lane.points,
                              std::max(0.015f, 0.025f * lane.probability),
                              0.0f, max_distance, openpilot_lane_color(lane.probability),
                              projection, logical_width, logical_height, rotate_landscape);
        }

        for (const ParsedRoadEdge &edge : output.road_edges) {
            if (!edge.valid) continue;
            const float confidence = std::clamp(1.0f - edge.std, 0.0f, 1.0f);
            if (confidence < 0.05f) continue;
            draw_model_ribbon(frame, edge.points,
                              0.025f, 0.0f, max_distance,
                              bgra(60, 60, 255, static_cast<int>(confidence * 200.0f)),
                              projection, logical_width, logical_height, rotate_landscape);
        }

        ParsedLeadPoint lead;
        float lead_probability = 0.0f;
        if (output.leads.primary(kLeadTimeIndex, kLeadProbabilityThreshold,
                                 &lead, &lead_probability)) {
            int px = 0;
            int py = 0;
            if (project_point(projection, lead.x, lead.y, kModelHeight,
                              logical_width, logical_height, &px, &py)) {
                rotate_model_point_180(logical_width, logical_height, &px, &py);
                const int size = std::clamp(static_cast<int>(13.0f - lead.x * 0.05f),
                                            7, 11);
                const int marker_y = py + size + 3;
                const float relative_speed_mps = lead.velocity - hud.speed_kph / 3.6f;
                draw_lead_chevron_180(frame, px, marker_y, size, lead.x, relative_speed_mps,
                                      lead_probability, logical_width, logical_height,
                                      rotate_landscape);
            }
        }
    }

    if (hud.left_blinker != previous_left_blinker_ ||
        hud.right_blinker != previous_right_blinker_) {
        blinker_blinking_rate_ = 120;
    }
    draw_turn_signals(frame, hud, blinker_blinking_rate_,
                      logical_width, logical_height, rotate_landscape);
    draw_hud(frame, hud, output, logical_width, logical_height, rotate_landscape);

    if (hud.left_blinker || hud.right_blinker) {
        blinker_blinking_rate_ -= 5;
        if (blinker_blinking_rate_ < 0)
            blinker_blinking_rate_ = 120;
    } else {
        blinker_blinking_rate_ = 120;
    }
    previous_left_blinker_ = hud.left_blinker;
    previous_right_blinker_ = hud.right_blinker;
}
