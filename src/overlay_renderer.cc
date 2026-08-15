#include "overlay_renderer.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

namespace {

struct EngageBlockLabel {
    const char *reason;
    const char *label;
};

constexpr EngageBlockLabel kEngageBlockLabels[] = {
    {"brake_error", "BRAKE ERROR"},
    {"control_stale", "CONTROL STALE"},
    {"controller_disabled", "CONTROL OFF"},
    {"door_open", "DOOR OPEN"},
    {"esp_disabled", "ESP OFF"},
    {"esp_stale", "ESP STALE"},
    {"gear_not_drive", "GEAR NOT D"},
    {"lanechange_manual", "MANUAL TURN"},
    {"lateral_plan_invalid", "PLAN INVALID"},
    {"lateral_plan_stale", "PLAN STALE"},
    {"mdps_fault", "MDPS FAULT"},
    {"no_smart_mdps_low_speed", "LOW SPEED"},
    {"not_engaged", "STANDBY"},
    {"panda_controls_off", "PANDA CTRL OFF"},
    {"panda_not_ready", "PANDA NOT READY"},
    {"park_brake", "PARK BRAKE"},
    {"path_invalid", "PATH INVALID"},
    {"seatbelt_unlatched", "SEATBELT"},
    {"seeds_missing", "CAN SEEDS"},
    {"speed_invalid", "SPEED INVALID"},
    {"steering_angle_limit", "ANGLE LIMIT"},
    {"vehicle_state_stale", "CAR STALE"},
    {"yaw_rate_invalid", "YAW INVALID"},
};

constexpr float kRadarToCameraDistanceM = 1.52f;
constexpr int kTrafficSpriteWidth = 270;
constexpr int kTrafficSpriteHeight = 155;

struct TrafficSignalSprite {
    cv::Mat logical;
    cv::Mat logical_mask;
    cv::Mat rotated;
    cv::Mat rotated_mask;

    bool load(const char *path)
    {
        logical = cv::imread(path, cv::IMREAD_UNCHANGED);
        if (logical.type() != CV_8UC4 || logical.cols != kTrafficSpriteWidth ||
            logical.rows != kTrafficSpriteHeight) {
            logical.release();
            return false;
        }
        cv::extractChannel(logical, logical_mask, 3);
        cv::rotate(logical, rotated, cv::ROTATE_90_CLOCKWISE);
        cv::rotate(logical_mask, rotated_mask, cv::ROTATE_90_CLOCKWISE);
        return true;
    }
};

struct TrafficSignalSprites {
    TrafficSignalSprite red;
    TrafficSignalSprite green;
    bool ready = false;

    TrafficSignalSprites()
    {
        ready = red.load("assets/ui/traffic_wait_red_retro-270x155-v3.png") &&
                green.load("assets/ui/traffic_go_green_retro-270x155-v3.png");
        if (!ready)
            std::fprintf(stderr, "overlay: traffic signal PNG assets unavailable\n");
    }
};

TrafficSignalSprites &traffic_signal_sprites()
{
    static TrafficSignalSprites sprites;
    return sprites;
}

void blit_traffic_signal(cv::Mat &frame, const TrafficSignalSprite &sprite,
                         int logical_x, int logical_y, int logical_height,
                         bool rotate_landscape)
{
    const cv::Mat &image = rotate_landscape ? sprite.rotated : sprite.logical;
    const cv::Mat &mask = rotate_landscape ? sprite.rotated_mask : sprite.logical_mask;
    const cv::Rect target = rotate_landscape
        ? cv::Rect(logical_height - logical_y - kTrafficSpriteHeight,
                   logical_x, kTrafficSpriteHeight, kTrafficSpriteWidth)
        : cv::Rect(logical_x, logical_y, kTrafficSpriteWidth, kTrafficSpriteHeight);
    if ((target & cv::Rect(0, 0, frame.cols, frame.rows)) != target) return;
    image.copyTo(frame(target), mask);
}

float display_lead_distance_m(const OverlayHudState &hud,
                              const ParsedLeadPoint *vision_lead)
{
    if (hud.radar_lead_valid && std::isfinite(hud.radar_lead_distance_m) &&
        hud.radar_lead_distance_m > 0.0f)
        return hud.radar_lead_distance_m;
    return vision_lead
        ? std::max(0.0f, vision_lead->x - kRadarToCameraDistanceM)
        : 0.0f;
}

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

const char *gear_text(int gear)
{
    switch (gear) {
    case 0: return "P";
    case 5: return "D";
    case 6: return "N";
    case 7: return "R";
    case 8: return "S";
    default: return "--";
    }
}

std::string active_block_text(const OverlayHudState &hud)
{
    if (hud.controller_active) return "ACTIVE";

    const std::string block = hud.active_block;
    if (block.empty()) return hud.controller_engaged ? "READY" : "STANDBY";
    if (const char *label = engage_block_label(block.c_str())) return label;

    std::string fallback = block;
    std::replace(fallback.begin(), fallback.end(), '_', ' ');
    return fallback;
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

    void box(int x, int y, int w, int h, uint32_t accent)
    {
        fill_rect(x, y, w, h, argb(82, 3, 7, 11));
        fill_rect(x, y, w, 1, argb(68, 255, 255, 255));
        fill_rect(x, y + h - 1, w, 1, argb(44, 255, 255, 255));
        fill_rect(x, y, 1, h, argb(52, 255, 255, 255));
        fill_rect(x + w - 1, y, 1, h, argb(34, 255, 255, 255));
        fill_rect(x, y, 3, h, accent);
    }

    void panel(int x, int y, int w, int h, uint32_t accent)
    {
        box(x, y, w, h, accent);
        fill_rect(x + 8, y + 17, w - 16, 1, argb(42, 255, 255, 255));
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

    constexpr int kCenterY = 42;
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

void draw_stop_signal_indicator(cv::Mat &img, const OverlayHudState &hud,
                                int logical_width, int logical_height,
                                bool rotate_landscape)
{
    const bool green =
        hud.departure_alert_type == DepartureAlertType::green_light;
    const bool red = hud.green_light_alert_armed;
    if (!green && !red) return;

    constexpr int kRightOffset = 126;
    constexpr int kSpriteCenterY = 320;
    constexpr int kHousingCenterX = 174;
    constexpr int kHousingCenterY = 39;
    const int center_x = logical_width - kRightOffset;

    const TrafficSignalSprites &sprites = traffic_signal_sprites();
    if (!sprites.ready) return;
    blit_traffic_signal(img, green ? sprites.green : sprites.red,
                        center_x - kHousingCenterX,
                        kSpriteCenterY - kHousingCenterY,
                        logical_height, rotate_landscape);
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
    constexpr int panel_h = 66;
    constexpr int panel_gap = 8;
    constexpr int panel_y0 = 10;
    constexpr int panel_y1 = panel_y0 + panel_h + panel_gap;
    constexpr int panel_y2 = panel_y1 + panel_h + panel_gap;
    constexpr int panel_y3 = panel_y2 + panel_h + panel_gap;
    auto draw_panel = [&](int x, int y, uint32_t accent, const char *title) {
        ui.panel(x, y, box_w, panel_h, accent);
        ui.hud_text_left(x + 10, y + 5, title, 1, dim, col_w);
    };
    constexpr int metric_inner_margin = 8;
    constexpr int metric_inner_w = box_w - 2 * metric_inner_margin;
    auto draw_metric_separators = [&](int x, int y, int columns) {
        const int column_w = metric_inner_w / columns;
        for (int column = 1; column < columns; ++column) {
            ui.fill_rect(x + metric_inner_margin + column * column_w,
                         y + 22, 1, 36, argb(38, 255, 255, 255));
        }
    };

    ui.hud_text_center(width / 2, 12,
                       format_text("%.0F", std::max(0.0f, hud.speed_kph)),
                       8, white, 220);
    ui.hud_text_center(width / 2, 80, "KPH", 2, dim, 140);

    const uint32_t control_color = hud.steering_fault ? red :
                                   (hud.controller_active ? green :
                                    (hud.controller_engaged ? yellow : dim));
    const uint32_t lateral_mode_color = !hud.lateral_mode_available ? dim :
                                        (hud.laneless_mode ? blue : green);
    const char *op_status = hud.controller_active ? "OP ACT" :
                            (hud.controller_engaged ? "OP EN" :
                             (hud.controller_enabled ? "OP RDY" : "OP OFF"));
    const uint32_t panda_color = !hud.panda_connected || !hud.panda_healthy ? orange :
                                 (hud.panda_faults != 0 ? red : green);

    draw_panel(left_box_x, panel_y0, status, "OPENPILOT");
    ui.hud_text_left(left_x, panel_y0 + 21, op_status, 3,
                     control_color, col_w);
    ui.hud_text_left(left_x, panel_y0 + 51,
                     format_text("PANDA %s  CAR %s",
                                 hud.panda_connected && hud.panda_healthy ? "OK" : "--",
                                 hud.vehicle_fresh ? "OK" : "--"),
                     1, panda_color, col_w);

    draw_panel(left_box_x, panel_y1, control_color, "CONTROL");
    constexpr int control_col_w = metric_inner_w / 4;
    draw_metric_separators(left_box_x, panel_y1, 4);
    const char *control_labels[] = {"ANGLE", "DES", "APPLY", "DRIVER"};
    const std::string control_values[] = {
        format_text("%.0F", hud.steering_angle_deg),
        format_text("%d", hud.desired_torque),
        format_text("%d", hud.apply_torque),
        format_text("%d", hud.driver_torque),
    };
    for (int col = 0; col < 4; ++col) {
        const int center_x = left_box_x + metric_inner_margin +
                             col * control_col_w + control_col_w / 2;
        ui.hud_text_center(center_x, panel_y1 + 22, control_labels[col],
                           1, dim, control_col_w - 4);
        ui.hud_text_center(center_x, panel_y1 + 39, control_values[col],
                           2, control_color, control_col_w - 4);
    }

    const uint32_t health_color =
        hud.cpu_percent >= 90.0f || hud.cpu_temp_c >= 85.0f || hud.storage_percent >= 95.0f ? red :
        (hud.cpu_percent >= 75.0f || hud.cpu_temp_c >= 75.0f || hud.storage_percent >= 85.0f ? orange :
         (hud.cpu_percent >= 60.0f || hud.cpu_temp_c >= 65.0f || hud.storage_percent >= 75.0f ? yellow : dim));
    const char *calibration_status = !hud.calibration_available ? "--" :
                                     (hud.calibration_status == 1 ? "OK" :
                                      (hud.calibration_status == 2 ? "BAD" : "WAIT"));
    const uint32_t calibration_color = !hud.calibration_available ? dim :
                                       (hud.calibration_status == 1 ? green :
                                        (hud.calibration_status == 2 ? red : yellow));
    const uint32_t network_color = !hud.network_connected ? orange :
        (hud.wifi_signal_dbm != 0 && hud.wifi_signal_dbm <= -75 ? yellow : green);
    const uint32_t system_color = hud.services_healthy ? blue : orange;
    draw_panel(right_box_x, panel_y0, system_color, "SYSTEM");
    ui.hud_text_left(right_x, panel_y0 + 21,
                     format_text("AI %.1F FPS  CAM %.1F FPS",
                                 hud.model_fps, hud.preview_fps),
                     1, dim, col_w);
    ui.hud_text_left(right_x, panel_y0 + 36,
                     format_text("HUD %.1F FPS", hud.overlay_fps),
                     1, dim, col_w);
    ui.hud_text_left(right_x, panel_y0 + 51,
                     hud.network_connected
                         ? (hud.wifi_signal_dbm != 0
                                ? format_text("NET %s %s %dDBM",
                                              hud.network_interface,
                                              hud.network_ipv4,
                                              hud.wifi_signal_dbm)
                                : format_text("NET %s %s",
                                              hud.network_interface,
                                              hud.network_ipv4))
                         : "NET OFFLINE",
                     1, network_color, col_w);

    draw_panel(right_box_x, panel_y1, health_color, "HEALTH");
    constexpr int health_col_w = metric_inner_w / 4;
    draw_metric_separators(right_box_x, panel_y1, 4);
    const char *health_labels[] = {"CPU", "TEMP", "MEM", "DISK"};
    const float health_values[] = {
        hud.cpu_percent, hud.cpu_temp_c, hud.memory_percent, hud.storage_percent,
    };
    for (int col = 0; col < 4; ++col) {
        const int center_x = right_box_x + metric_inner_margin +
                             col * health_col_w + health_col_w / 2;
        ui.hud_text_center(center_x, panel_y1 + 22, health_labels[col],
                           1, dim, health_col_w - 4);
        ui.hud_text_center(center_x, panel_y1 + 39,
                           col == 1 ? format_text("%.0FC", health_values[col])
                                    : format_text("%.0F%%", health_values[col]),
                           2, health_color, health_col_w - 4);
    }

    draw_panel(right_box_x, panel_y2, calibration_color, "CALIBRATION");
    ui.hud_text_center(right_box_x + box_w - 37, panel_y2 + 5,
                       format_text("%s B%d", calibration_status,
                                   std::max(0, hud.calibration_valid_blocks)),
                       1, calibration_color, 68);
    const int calibration_inner_x = right_box_x + metric_inner_margin;
    constexpr int calibration_col_w = metric_inner_w / 3;
    draw_metric_separators(right_box_x, panel_y2, 3);
    const char *calibration_labels[] = {"ROLL", "PITCH", "YAW"};
    const float calibration_values[] = {
        hud.calibration_roll_deg,
        hud.calibration_pitch_deg,
        hud.calibration_yaw_deg,
    };
    for (int col = 0; col < 3; ++col) {
        const int center_x = calibration_inner_x + col * calibration_col_w +
                             calibration_col_w / 2;
        ui.hud_text_center(center_x, panel_y2 + 22, calibration_labels[col],
                           1, dim, calibration_col_w - 6);
        ui.hud_text_center(center_x, panel_y2 + 39,
                           format_text("%.2F", calibration_values[col]),
                           2, calibration_color, calibration_col_w - 6);
    }

    constexpr int drive_box_y = panel_y2;
    const bool maximum_speed_valid =
        std::isfinite(hud.cruise_max_speed_kph) &&
        hud.cruise_max_speed_kph > 0.0f;
    const bool command_speed_valid =
        std::isfinite(hud.cruise_command_speed_kph) &&
        hud.cruise_command_speed_kph > 0.0f;
    draw_panel(left_box_x, drive_box_y, control_color, "DRIVE");
    ui.hud_text_center(left_box_x + box_w - 37, drive_box_y + 5,
                       !hud.lateral_mode_available ? "--" :
                           (hud.laneless_mode ? "LANELESS" : "LANE"),
                       1, lateral_mode_color, 68);
    ui.hud_text_left(left_x, drive_box_y + 22,
                     maximum_speed_valid && command_speed_valid
                         ? format_text("MAX %.0F  SET %.0F",
                                       hud.cruise_max_speed_kph,
                                       hud.cruise_command_speed_kph)
                         : "MAX --  SET --",
                     2, control_color, col_w);
    ui.hud_text_left(left_x, drive_box_y + 47,
                     format_text("GEAR %s  CRZ %s  %s",
                                 gear_text(hud.gear),
                                 hud.cruise_active ? "ON" : "OFF",
                                 active_block_text(hud).c_str()),
                     1, control_color, col_w);

    ParsedLeadPoint lead;
    float lead_probability = 0.0f;
    const bool have_lead = output.valid &&
                           output.leads.primary(0, 0.5f, &lead, &lead_probability);
    const bool have_radar_lead =
        hud.radar_lead_valid && std::isfinite(hud.radar_lead_distance_m) &&
        hud.radar_lead_distance_m > 0.0f;
    const bool have_display_lead = have_radar_lead || have_lead;
    const float lead_distance_m =
        display_lead_distance_m(hud, have_lead ? &lead : nullptr);
    const float relative_speed_kph = have_radar_lead
        ? hud.radar_lead_relative_speed_mps * 3.6f
        : (have_lead ? (lead.velocity - hud.ego_speed_kph / 3.6f) * 3.6f : 0.0f);
    const uint32_t lead_color = !have_display_lead ? dim :
        ((lead_distance_m < 15.0f || relative_speed_kph < -20.0f)
             ? orange : blue);
    constexpr int lead_box_y = panel_y3;
    draw_panel(right_box_x, lead_box_y, lead_color, "LEAD");
    if (have_display_lead) {
        ui.hud_text_left(right_x, lead_box_y + 22,
                         format_text("DIST %.0FM P %.0F%%",
                                     lead_distance_m,
                                     have_lead ? lead_probability * 100.0f : 0.0f),
                         2, lead_color, col_w);
        ui.hud_text_left(right_x, lead_box_y + 47,
                         format_text("REL %+.0F KPH", relative_speed_kph),
                         2, lead_color, col_w);
    } else {
        ui.hud_text_left(right_x, lead_box_y + 22, "NO LEAD", 2, dim, col_w);
        ui.hud_text_left(right_x, lead_box_y + 47, "REL -- KPH", 2, dim, col_w);
    }

    draw_stop_signal_indicator(frame, hud,
                               logical_width, logical_height, rotate_landscape);

    if (hud.brake_hold) {
        constexpr int auto_hold_w = 190;
        constexpr int auto_hold_h = 40;
        constexpr int auto_hold_y = 350;
        const int auto_hold_x = (width - auto_hold_w) / 2;
        ui.box(auto_hold_x, auto_hold_y, auto_hold_w, auto_hold_h, green);
        ui.hud_text_center(width / 2, auto_hold_y + 10, "AUTO HOLD", 3,
                           green, auto_hold_w - 18);
    }

    constexpr int tpms_box_y = panel_y3;
    constexpr int tpms_col_offset = 112;
    constexpr int tpms_col_w = 108;
    const bool tpms_bar = hud.tpms_unit == 2;
    const float tpms_low = tpms_bar ? 2.2f : 32.0f;
    const float tpms_high = tpms_bar ? 2.8f : 45.0f;
    auto pressure_available = [&](float pressure) {
        return hud.tpms_valid && std::isfinite(pressure) && pressure > 0.0f;
    };
    auto pressure_color = [&](float pressure) {
        if (!pressure_available(pressure)) return dim;
        if (pressure > tpms_high) return red;
        if (pressure < tpms_low) return yellow;
        return green;
    };
    auto pressure_text = [&](const char *wheel, float pressure) {
        if (!pressure_available(pressure))
            return format_text("%s -", wheel);
        return tpms_bar ? format_text("%s %.1F", wheel, pressure)
                        : format_text("%s %.0F", wheel, pressure);
    };
    const std::array<float, 4> pressures = {
        hud.tpms_pressure_fl, hud.tpms_pressure_fr,
        hud.tpms_pressure_rl, hud.tpms_pressure_rr,
    };
    bool tpms_low_pressure = false;
    bool tpms_high_pressure = false;
    for (float pressure : pressures) {
        if (!pressure_available(pressure)) continue;
        tpms_low_pressure |= pressure < tpms_low;
        tpms_high_pressure |= pressure > tpms_high;
    }
    const uint32_t tpms_color = !hud.tpms_valid ? dim :
        ((hud.tpms_warning || tpms_high_pressure) ? red :
         (tpms_low_pressure ? yellow : green));
    draw_panel(left_box_x, tpms_box_y, tpms_color, "TPMS");
    ui.hud_text_left(left_x + 178, tpms_box_y + 5,
                     tpms_bar ? "BAR" : "PSI", 1, tpms_color, 38);
    ui.fill_rect(left_x + tpms_col_offset - 8, tpms_box_y + 22, 1, 38,
                 argb(38, 255, 255, 255));
    ui.hud_text_left(left_x, tpms_box_y + 22,
                     pressure_text("FL", hud.tpms_pressure_fl), 2,
                     pressure_color(hud.tpms_pressure_fl), tpms_col_w);
    ui.hud_text_left(left_x + tpms_col_offset, tpms_box_y + 22,
                     pressure_text("FR", hud.tpms_pressure_fr), 2,
                     pressure_color(hud.tpms_pressure_fr), tpms_col_w);
    ui.hud_text_left(left_x, tpms_box_y + 43,
                     pressure_text("RL", hud.tpms_pressure_rl), 2,
                     pressure_color(hud.tpms_pressure_rl), tpms_col_w);
    ui.hud_text_left(left_x + tpms_col_offset, tpms_box_y + 43,
                     pressure_text("RR", hud.tpms_pressure_rr), 2,
                     pressure_color(hud.tpms_pressure_rr), tpms_col_w);

    std::string alert;
    uint32_t alert_color = orange;
    bool showing_departure_alert = false;
    if (hud.engage_alert_message[0] != '\0') {
        alert = hud.engage_alert_message;
        alert_color = orange;
    } else if (hud.steering_fault) {
        alert = "STEERING FAULT";
        alert_color = red;
    } else if (hud.panda_faults != 0) {
        alert = "PANDA FAULT";
        alert_color = red;
    } else if (!hud.services_healthy) {
        alert = "WAITING FOR SERVICES";
    } else if (hud.departure_alert_type == DepartureAlertType::lead_departed) {
        alert = "LEAD VEHICLE MOVING";
        alert_color = green;
        showing_departure_alert = true;
    } else if (hud.departure_alert_type == DepartureAlertType::green_light) {
        alert = "TRAFFIC SIGNAL CHANGED";
        alert_color = green;
        showing_departure_alert = true;
    }
    if (!alert.empty()) {
        constexpr int alert_w = 520;
        const int alert_x = (width - alert_w) / 2;
        const int alert_y = height - 58;
        ui.box(alert_x, alert_y, alert_w, 50, alert_color);
        ui.hud_text_center(width / 2, alert_y + 8, alert, 3,
                           alert_color, alert_w - 18);
        ui.hud_text_center(width / 2, alert_y + 36,
                           showing_departure_alert
                               ? "CHECK ROAD AND PROCEED"
                               : format_text("M%s CTL%s PND%s",
                                             output.valid ? "OK" : "--",
                                             hud.vehicle_fresh ? "OK" : "--",
                                             hud.panda_connected ? "OK" : "--"),
                           1, white, alert_w - 18);
    }
}

bool project_display_point(const ModelPoint &point, float y_offset, float z_offset,
                           const ProjectionState &projection,
                           int logical_width, int logical_height,
                           bool rotate_landscape, cv::Point *projected)
{
    if (projected == nullptr || !std::isfinite(point.x) ||
        !std::isfinite(point.y) || !std::isfinite(point.z)) {
        return false;
    }

    int px = 0;
    int py = 0;
    if (!project_point(projection, point.x, point.y + y_offset, point.z + z_offset,
                       logical_width, logical_height, &px, &py)) {
        return false;
    }

    rotate_model_point_180(logical_width, logical_height, &px, &py);
    *projected = display_point(px, py, logical_height, rotate_landscape);
    return true;
}

void draw_model_ribbon(cv::Mat &img,
                       const std::array<ModelPoint, kTrajectorySize> &points,
                       float half_width, float z_offset, float max_distance,
                       const cv::Scalar &color, const ProjectionState &projection,
                       int logical_width, int logical_height, bool rotate_landscape)
{
    struct ProjectedPair {
        cv::Point left;
        cv::Point right;
    };

    std::vector<ProjectedPair> pairs;
    pairs.reserve(kTrajectorySize);
    float previous_x = -1.0f;
    double polygon_winding = 0.0;

    for (const ModelPoint &point : points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            if (!pairs.empty()) break;
            continue;
        }
        if (point.x > max_distance) break;
        if (point.x < 0.5f) continue;
        if (!pairs.empty() && point.x <= previous_x) break;

        ProjectedPair pair;
        const bool projected =
            project_display_point(point, -half_width, z_offset, projection,
                                  logical_width, logical_height, rotate_landscape,
                                  &pair.left) &&
            project_display_point(point, half_width, z_offset, projection,
                                  logical_width, logical_height, rotate_landscape,
                                  &pair.right);
        if (!projected) {
            if (!pairs.empty()) break;
            continue;
        }

        if (!pairs.empty()) {
            const ProjectedPair &previous = pairs.back();
            const cv::Point quad[] = {
                previous.left, pair.left, pair.right, previous.right,
            };
            double area = 0.0;
            for (int i = 0; i < 4; ++i) {
                const cv::Point &a = quad[i];
                const cv::Point &b = quad[(i + 1) % 4];
                area += static_cast<double>(a.x) * b.y -
                        static_cast<double>(b.x) * a.y;
            }
            area *= 0.5;
            if (std::abs(area) < 0.5 ||
                (polygon_winding != 0.0 && area * polygon_winding <= 0.0)) {
                break;
            }
            if (polygon_winding == 0.0) polygon_winding = area;
        }

        pairs.push_back(pair);
        previous_x = point.x;
    }

    if (pairs.size() < 2) return;

    std::vector<cv::Point> vertices;
    vertices.reserve(pairs.size() * 2);
    for (const ProjectedPair &pair : pairs) vertices.push_back(pair.left);
    for (auto it = pairs.rbegin(); it != pairs.rend(); ++it)
        vertices.push_back(it->right);

    const cv::Point *polygon[] = {vertices.data()};
    const int count[] = {static_cast<int>(vertices.size())};
    cv::fillPoly(img, polygon, count, 1, color, cv::LINE_8);
}

cv::Scalar openpilot_path_color(const OverlayHudState &hud)
{
    if (!hud.controller_engaged)
        return bgra(255, 255, 255, 150);
    if (!hud.controller_active)
        return bgra(0, 210, 255, 160);

    const float output_scale = std::clamp(std::abs(hud.normalized_output) * 0.9f,
                                          0.0f, 1.0f);
    const int red = static_cast<int>(output_scale * 255.0f);
    const int green = static_cast<int>((1.0f - output_scale) * 255.0f);
    if (hud.laneless_mode)
        return bgra(green, 150, red, 160);
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

const char *engage_block_label(const char *block)
{
    if (!block || block[0] == '\0') return nullptr;
    for (const EngageBlockLabel &entry : kEngageBlockLabels) {
        if (std::strcmp(block, entry.reason) == 0) return entry.label;
    }
    return nullptr;
}

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

        if (!hud.laneless_mode) {
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
                const float lead_distance_m = display_lead_distance_m(hud, &lead);
                const int size = std::clamp(
                    static_cast<int>(13.0f - lead_distance_m * 0.05f),
                    7, 11);
                const int marker_y = py + size + 3;
                const float relative_speed_mps =
                    lead.velocity - hud.ego_speed_kph / 3.6f;
                draw_lead_chevron_180(frame, px, marker_y, size, lead_distance_m,
                                      relative_speed_mps,
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
