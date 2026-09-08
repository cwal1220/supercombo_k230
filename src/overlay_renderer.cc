#include "overlay_renderer.h"

#include "common_utils.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

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
    {"stopped", "STOPPED"},
    {"steering_angle_limit", "ANGLE LIMIT"},
    {"vehicle_state_stale", "CAR STALE"},
    {"yaw_rate_invalid", "YAW INVALID"},
};

constexpr uint32_t argb(uint8_t a, uint8_t r, uint8_t g, uint8_t b)
{
    return (static_cast<uint32_t>(a) << 24) |
           (static_cast<uint32_t>(r) << 16) |
           (static_cast<uint32_t>(g) << 8) |
           static_cast<uint32_t>(b);
}

cv::Scalar bgra(int b, int g, int r, int a = 255)
{
    return cv::Scalar(b, g, r, a);
}

/* 팔레트. */
constexpr uint32_t kWhite = argb(230, 255, 255, 255);
constexpr uint32_t kDim = argb(170, 210, 220, 230);
constexpr uint32_t kGreen = argb(230, 80, 230, 95);
constexpr uint32_t kBlue = argb(230, 90, 170, 255);
constexpr uint32_t kYellow = argb(230, 255, 220, 60);
constexpr uint32_t kOrange = argb(235, 255, 150, 50);
constexpr uint32_t kRed = argb(235, 255, 70, 70);
constexpr uint32_t kShadow = argb(185, 0, 0, 0);
constexpr uint32_t kPanelFill = argb(82, 3, 7, 11);
constexpr uint32_t kPanelEdgeTop = argb(68, 255, 255, 255);
constexpr uint32_t kPanelEdgeBottom = argb(44, 255, 255, 255);
constexpr uint32_t kPanelEdgeLeft = argb(52, 255, 255, 255);
constexpr uint32_t kPanelEdgeRight = argb(34, 255, 255, 255);
constexpr uint32_t kPanelRule = argb(42, 255, 255, 255);
constexpr uint32_t kSeparator = argb(38, 255, 255, 255);

/* 글꼴: 5x7 글리프, 6칸 전진, 그림자는 오른쪽 아래 2 px. */
constexpr int kGlyphW = 5;
constexpr int kGlyphH = 7;
constexpr int kGlyphAdvance = 6;
constexpr int kShadowOffset = 2;

/* 패널 격자: 좌우 각 4단, 800x480 논리 좌표. */
constexpr int kBoxW = 236;
constexpr int kBoxMargin = 8;
constexpr int kBoxPadding = 10;
constexpr int kColW = kBoxW - 2 * kBoxPadding;
constexpr int kPanelH = 66;
constexpr int kPanelGap = 8;
constexpr int kPanelY0 = 10;
constexpr int kPanelY1 = kPanelY0 + kPanelH + kPanelGap;
constexpr int kPanelY2 = kPanelY1 + kPanelH + kPanelGap;
constexpr int kPanelY3 = kPanelY2 + kPanelH + kPanelGap;
constexpr int kLeftBoxX = kBoxMargin;
constexpr int kPanelAccentW = 3;
constexpr int kPanelRuleY = 17;
constexpr int kPanelRuleInset = 8;
constexpr int kMetricInnerMargin = 8;
constexpr int kMetricInnerW = kBoxW - 2 * kMetricInnerMargin;
constexpr int kMetricTextMargin = 4;

/* 패널 안 행 위치(패널 상단 기준). 제목줄 아래에 scale 2 두 줄(Row1/Row2) 또는
 * scale 1 세 줄(Row1/Line2/Line3). */
constexpr int kPanelTitleY = 5;
constexpr int kPanelRow1Y = 22;
constexpr int kPanelRow2Y = 47;
constexpr int kPanelLine2Y = 36;
constexpr int kPanelLine3Y = 51;
constexpr int kMetricValueY = 39;
constexpr int kMetricSeparatorH = 36;
constexpr int kBadgeCenterInset = 37;
constexpr int kBadgeMaxW = 68;
constexpr int kBoxTextInset = 18;

/* 상태 띠와 중앙 속도. */
constexpr int kStatusBarH = 6;
constexpr int kSpeedY = 12;
constexpr int kSpeedScale = 8;
constexpr int kSpeedMaxW = 220;
constexpr int kSpeedUnitY = 80;

/* AUTO HOLD 상자와 하단 알림 상자. */
constexpr int kAutoHoldW = 190;
constexpr int kAutoHoldH = 40;
constexpr int kAutoHoldY = 350;
constexpr int kAutoHoldTextY = 10;
constexpr int kAlertW = 520;
constexpr int kAlertH = 50;
constexpr int kAlertBottomMargin = 58;
constexpr int kAlertTitleY = 8;
constexpr int kAlertDetailY = 36;

/* TPMS 패널: 두 열, 단위는 제목줄 배지. */
constexpr int kTpmsColumnOffset = 112;
constexpr int kTpmsColumnW = 108;
constexpr float kTpmsLowBar = 2.2f;
constexpr float kTpmsHighBar = 2.8f;
constexpr float kTpmsLowPsi = 32.0f;
constexpr float kTpmsHighPsi = 45.0f;

/* 리드: 패널 경고 색 기준과 장면 마커. */
constexpr float kRadarToCameraDistanceM = 1.52f;
constexpr float kLeadProbabilityThreshold = 0.5f;
constexpr int kLeadTimeIndex = 0;
constexpr float kLeadCloseDistanceM = 15.0f;
constexpr float kLeadClosingKph = -20.0f;
constexpr float kLeadMarkerSizeBase = 13.0f;
constexpr float kLeadMarkerSizePerMeter = 0.05f;
constexpr int kLeadMarkerSizeMin = 7;
constexpr int kLeadMarkerSizeMax = 11;
constexpr int kLeadMarkerOffsetY = 3;
constexpr float kLeadRiskDistanceM = 40.0f;
constexpr float kLeadRiskClosingMps = 10.0f;

/* 신호등 스프라이트: 하우징 중심을 오른쪽 여백 안에 둔다. */
constexpr int kTrafficSpriteWidth = 270;
constexpr int kTrafficSpriteHeight = 155;
constexpr int kSignalRightOffset = 126;
constexpr int kSignalCenterY = 320;
constexpr int kSignalHousingCenterX = 174;
constexpr int kSignalHousingCenterY = 39;

/* 깜빡이 셰브런: 단계 0~14 켜짐(4부터 2개, 8부터 3개), 15~24 꺼짐. */
constexpr int kTurnCenterY = 42;
constexpr int kTurnInnerOffset = 74;
constexpr int kTurnChevronStep = 28;
constexpr int kTurnChevronW = 24;
constexpr int kTurnChevronHalfH = 34;
constexpr int kTurnLitSteps = 15;
constexpr int kTurnChevronStartStep[] = {0, 4, 8};
constexpr int kTurnChevronAlpha[] = {70, 140, 210};

/* 장면 그리기 거리와 폭. */
constexpr float kMinDrawDistance = 10.0f;
constexpr float kMaxDrawDistance = 100.0f;
constexpr float kPathHalfWidth = 0.9f;
constexpr float kLaneHalfWidthMin = 0.015f;
constexpr float kLaneHalfWidthPerProbability = 0.025f;
constexpr float kEdgeHalfWidth = 0.025f;
constexpr float kLaneMinProbability = 0.05f;
constexpr float kEdgeMinConfidence = 0.05f;

constexpr int kWeakWifiDbm = -75;

struct HealthLevel {
    float cpu_percent;
    float temp_c;
    float storage_percent;
    uint32_t color;
};

/* 위에서부터 첫 일치가 색을 정한다. */
constexpr HealthLevel kHealthLevels[] = {
    {90.0f, 85.0f, 95.0f, kRed},
    {75.0f, 75.0f, 85.0f, kOrange},
    {60.0f, 65.0f, 75.0f, kYellow},
};

/* 한 프레임의 그리기 대상: 네이티브 버퍼와 800x480 논리 좌표계. */
struct Frame {
    cv::Mat &mat;
    int width;
    int height;
    bool rotated;

    cv::Point native(int x, int y) const
    {
        return rotated ? cv::Point(height - 1 - y, x) : cv::Point(x, y);
    }
};

struct TrafficSignalSprite {
    cv::Mat logical;
    cv::Mat logical_mask;
    cv::Mat rotated;
    cv::Mat rotated_mask;

    bool load(const std::string &path)
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

/* 스프라이트가 화면에 다 들어올 때만 그린다. 마스크(α)가 0인 픽셀은 건너뛴다. */
void blit_traffic_signal(const Frame &frame, const TrafficSignalSprite &sprite,
                         int logical_x, int logical_y)
{
    const cv::Mat &image = frame.rotated ? sprite.rotated : sprite.logical;
    const cv::Mat &mask = frame.rotated ? sprite.rotated_mask : sprite.logical_mask;
    const cv::Rect target = frame.rotated
        ? cv::Rect(frame.height - logical_y - kTrafficSpriteHeight,
                   logical_x, kTrafficSpriteHeight, kTrafficSpriteWidth)
        : cv::Rect(logical_x, logical_y, kTrafficSpriteWidth, kTrafficSpriteHeight);
    if ((target & cv::Rect(0, 0, frame.mat.cols, frame.mat.rows)) != target) return;
    image.copyTo(frame.mat(target), mask);
}

} // namespace

struct TrafficSignalSprites {
    TrafficSignalSprite red;
    TrafficSignalSprite green;
};

namespace {

template <typename... Args>
std::string format_text(const char *format, Args... args)
{
    char text[128];
    std::snprintf(text, sizeof(text), format, args...);
    return text;
}

/* ---- 상태 → 색·문구 ---- */

uint32_t status_color(const OverlayHudState &hud)
{
    if (hud.steering_fault || hud.panda_faults != 0) return kRed;
    if (!hud.services_healthy) return kOrange;
    if (hud.controller_active) return kGreen;
    return hud.controller_enabled ? kBlue : kDim;
}

uint32_t control_color(const OverlayHudState &hud)
{
    if (hud.steering_fault) return kRed;
    if (hud.controller_active) return kGreen;
    return hud.controller_engaged ? kYellow : kDim;
}

uint32_t system_color(const OverlayHudState &hud)
{
    return hud.services_healthy ? kBlue : kOrange;
}

uint32_t panda_color(const OverlayHudState &hud)
{
    if (!hud.panda_connected || !hud.panda_healthy) return kOrange;
    return hud.panda_faults != 0 ? kRed : kGreen;
}

uint32_t network_color(const OverlayHudState &hud)
{
    if (!hud.network_connected) return kOrange;
    return hud.wifi_signal_dbm != 0 && hud.wifi_signal_dbm <= kWeakWifiDbm ? kYellow : kGreen;
}

uint32_t health_color(const OverlayHudState &hud)
{
    for (const HealthLevel &level : kHealthLevels) {
        if (hud.cpu_percent >= level.cpu_percent || hud.cpu_temp_c >= level.temp_c ||
            hud.storage_percent >= level.storage_percent)
            return level.color;
    }
    return kDim;
}

uint32_t calibration_color(const OverlayHudState &hud)
{
    if (!hud.calibration_available) return kDim;
    if (hud.calibration_status == 1) return kGreen;
    return hud.calibration_status == 2 ? kRed : kYellow;
}

uint32_t lateral_mode_color(const OverlayHudState &hud)
{
    if (!hud.lateral_mode_available) return kDim;
    return hud.laneless_mode ? kBlue : kGreen;
}

const char *op_status_text(const OverlayHudState &hud)
{
    if (hud.controller_active) return "OP ACT";
    if (hud.controller_engaged) return "OP EN";
    return hud.controller_enabled ? "OP RDY" : "OP OFF";
}

const char *calibration_status_text(const OverlayHudState &hud)
{
    if (!hud.calibration_available) return "--";
    if (hud.calibration_status == 1) return "OK";
    return hud.calibration_status == 2 ? "BAD" : "WAIT";
}

const char *lateral_mode_text(const OverlayHudState &hud)
{
    if (!hud.lateral_mode_available) return "--";
    return hud.laneless_mode ? "LANELESS" : "LANE";
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

std::string network_text(const OverlayHudState &hud)
{
    if (!hud.network_connected) return "NET OFFLINE";
    if (hud.wifi_signal_dbm != 0)
        return format_text("NET %s %s %dDBM", hud.network_interface, hud.network_ipv4,
                           hud.wifi_signal_dbm);
    return format_text("NET %s %s", hud.network_interface, hud.network_ipv4);
}

std::string cruise_text(const OverlayHudState &hud)
{
    const bool maximum_valid =
        std::isfinite(hud.cruise_max_speed_kph) && hud.cruise_max_speed_kph > 0.0f;
    const bool command_valid =
        std::isfinite(hud.cruise_command_speed_kph) && hud.cruise_command_speed_kph > 0.0f;
    if (!maximum_valid || !command_valid) return "MAX --  SET --";
    return format_text("MAX %.0F  SET %.0F", hud.cruise_max_speed_kph,
                       hud.cruise_command_speed_kph);
}

/* ---- 리드 ---- */

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

/* LEAD 패널과 장면 마커가 공유한다. */
struct LeadInfo {
    bool vision = false;
    bool radar = false;
    ParsedLeadPoint point {};
    float probability = 0.0f;
    float distance_m = 0.0f;
    float relative_speed_kph = 0.0f;

    bool any() const { return radar || vision; }
};

LeadInfo lead_info(const OverlayHudState &hud, const ParsedModelOutput &output)
{
    LeadInfo info;
    info.vision = output.valid &&
                  output.leads.primary(kLeadTimeIndex, kLeadProbabilityThreshold,
                                       &info.point, &info.probability);
    info.radar = hud.radar_lead_valid && std::isfinite(hud.radar_lead_distance_m) &&
                 hud.radar_lead_distance_m > 0.0f;
    info.distance_m = display_lead_distance_m(hud, info.vision ? &info.point : nullptr);
    info.relative_speed_kph = info.radar
        ? hud.radar_lead_relative_speed_mps * 3.6f
        : (info.vision ? (info.point.velocity - hud.ego_speed_kph / 3.6f) * 3.6f : 0.0f);
    return info;
}

uint32_t lead_color(const LeadInfo &lead)
{
    if (!lead.any()) return kDim;
    return lead.distance_m < kLeadCloseDistanceM || lead.relative_speed_kph < kLeadClosingKph
        ? kOrange : kBlue;
}

/* ---- TPMS ---- */

struct TpmsRange {
    bool bar;
    float low;
    float high;
};

TpmsRange tpms_range(const OverlayHudState &hud)
{
    if (hud.tpms_unit == 2) return {true, kTpmsLowBar, kTpmsHighBar};
    return {false, kTpmsLowPsi, kTpmsHighPsi};
}

bool pressure_available(const OverlayHudState &hud, float pressure)
{
    return hud.tpms_valid && std::isfinite(pressure) && pressure > 0.0f;
}

uint32_t pressure_color(const OverlayHudState &hud, const TpmsRange &range, float pressure)
{
    if (!pressure_available(hud, pressure)) return kDim;
    if (pressure > range.high) return kRed;
    if (pressure < range.low) return kYellow;
    return kGreen;
}

std::string pressure_text(const OverlayHudState &hud, const TpmsRange &range,
                          const char *wheel, float pressure)
{
    if (!pressure_available(hud, pressure)) return format_text("%s -", wheel);
    return range.bar ? format_text("%s %.1F", wheel, pressure)
                     : format_text("%s %.0F", wheel, pressure);
}

uint32_t tpms_color(const OverlayHudState &hud, const TpmsRange &range)
{
    if (!hud.tpms_valid) return kDim;
    bool low = false;
    bool high = false;
    for (float pressure : {hud.tpms_pressure_fl, hud.tpms_pressure_fr,
                           hud.tpms_pressure_rl, hud.tpms_pressure_rr}) {
        if (!pressure_available(hud, pressure)) continue;
        low |= pressure < range.low;
        high |= pressure > range.high;
    }
    if (hud.tpms_warning || high) return kRed;
    return low ? kYellow : kGreen;
}

/* ---- 알림 ---- */

struct Alert {
    std::string title;
    uint32_t color = kOrange;
    bool departure = false;

    bool empty() const { return title.empty(); }
};

/* 우선순위: engage 거부 토스트 > 조향 결함 > panda 결함 > 서비스 대기 > 출발 감지. */
Alert select_alert(const OverlayHudState &hud)
{
    if (hud.engage_alert_message[0] != '\0') return {hud.engage_alert_message, kOrange, false};
    if (hud.steering_fault) return {"STEERING FAULT", kRed, false};
    if (hud.panda_faults != 0) return {"PANDA FAULT", kRed, false};
    if (!hud.services_healthy) return {"WAITING FOR SERVICES", kOrange, false};
    if (hud.departure_alert_type == DepartureAlertType::lead_departed)
        return {"LEAD VEHICLE MOVING", kGreen, true};
    if (hud.departure_alert_type == DepartureAlertType::green_light)
        return {"TRAFFIC SIGNAL CHANGED", kGreen, true};
    return {};
}

/* ---- 5x7 글꼴 ---- */

using GlyphRows = std::array<uint8_t, kGlyphH>;

struct Glyph {
    char c;
    GlyphRows rows;
};

constexpr char kGlyphFirst = ' ';
constexpr char kGlyphLast = '_';
constexpr int kGlyphCount = kGlyphLast - kGlyphFirst + 1;

/* 행마다 5비트, MSB가 왼쪽. ' '..'_' 밖은 공백. */
constexpr Glyph kGlyphs[] = {
    {'0', {0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e}},
    {'1', {0x04, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e}},
    {'2', {0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f}},
    {'3', {0x1e, 0x01, 0x01, 0x0e, 0x01, 0x01, 0x1e}},
    {'4', {0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02}},
    {'5', {0x1f, 0x10, 0x1e, 0x01, 0x01, 0x11, 0x0e}},
    {'6', {0x06, 0x08, 0x10, 0x1e, 0x11, 0x11, 0x0e}},
    {'7', {0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}},
    {'8', {0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e}},
    {'9', {0x0e, 0x11, 0x11, 0x0f, 0x01, 0x02, 0x0c}},
    {'A', {0x0e, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11}},
    {'B', {0x1e, 0x11, 0x11, 0x1e, 0x11, 0x11, 0x1e}},
    {'C', {0x0e, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0e}},
    {'D', {0x1e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1e}},
    {'E', {0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x1f}},
    {'F', {0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10}},
    {'G', {0x0e, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0f}},
    {'H', {0x11, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11}},
    {'I', {0x0e, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0e}},
    {'J', {0x07, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0c}},
    {'K', {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}},
    {'L', {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f}},
    {'M', {0x11, 0x1b, 0x15, 0x15, 0x11, 0x11, 0x11}},
    {'N', {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11}},
    {'O', {0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e}},
    {'P', {0x1e, 0x11, 0x11, 0x1e, 0x10, 0x10, 0x10}},
    {'Q', {0x0e, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0d}},
    {'R', {0x1e, 0x11, 0x11, 0x1e, 0x14, 0x12, 0x11}},
    {'S', {0x0f, 0x10, 0x10, 0x0e, 0x01, 0x01, 0x1e}},
    {'T', {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}},
    {'U', {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e}},
    {'V', {0x11, 0x11, 0x11, 0x11, 0x11, 0x0a, 0x04}},
    {'W', {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0a}},
    {'X', {0x11, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x11}},
    {'Y', {0x11, 0x11, 0x0a, 0x04, 0x04, 0x04, 0x04}},
    {'Z', {0x1f, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1f}},
    {'-', {0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00}},
    {'.', {0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x0c}},
    {':', {0x00, 0x04, 0x04, 0x00, 0x04, 0x04, 0x00}},
    {'/', {0x01, 0x02, 0x02, 0x04, 0x08, 0x08, 0x10}},
    {'%', {0x19, 0x1a, 0x02, 0x04, 0x08, 0x0b, 0x13}},
    {'<', {0x02, 0x04, 0x08, 0x10, 0x08, 0x04, 0x02}},
    {'>', {0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08}},
    {'!', {0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x04}},
    {'?', {0x0e, 0x11, 0x01, 0x02, 0x04, 0x00, 0x04}},
    {'+', {0x00, 0x04, 0x04, 0x1f, 0x04, 0x04, 0x00}},
    {'=', {0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00}},
    {'_', {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f}},
};

/* 글리프 행의 연속 픽셀 구간. 5비트 행은 최대 3구간, length 0이 끝. */
struct GlyphRun {
    uint8_t start = 0;
    uint8_t length = 0;
};
using GlyphRowRuns = std::array<GlyphRun, 3>;
using GlyphRuns = std::array<GlyphRowRuns, kGlyphH>;

constexpr GlyphRuns glyph_runs_from_rows(const GlyphRows &rows)
{
    GlyphRuns runs {};
    for (int row = 0; row < kGlyphH; ++row) {
        int count = 0;
        int col = 0;
        while (col < kGlyphW) {
            if (!(rows[row] & (1U << (kGlyphW - 1 - col)))) {
                ++col;
                continue;
            }
            int end = col + 1;
            while (end < kGlyphW && (rows[row] & (1U << (kGlyphW - 1 - end)))) ++end;
            runs[row][count++] = GlyphRun{static_cast<uint8_t>(col),
                                          static_cast<uint8_t>(end - col)};
            col = end;
        }
    }
    return runs;
}

constexpr std::array<GlyphRuns, kGlyphCount> kGlyphRunTable = [] {
    std::array<GlyphRuns, kGlyphCount> table {};
    for (const Glyph &glyph : kGlyphs)
        table[glyph.c - kGlyphFirst] = glyph_runs_from_rows(glyph.rows);
    return table;
}();

const GlyphRuns &glyph_runs(char c)
{
    static constexpr GlyphRuns blank {};
    if (c < kGlyphFirst || c > kGlyphLast) return blank;
    return kGlyphRunTable[c - kGlyphFirst];
}

/* ---- 비트맵 HUD 그리기 ---- */

class BitmapHud {
public:
    explicit BitmapHud(const Frame &frame) : frame_(frame) {}

    int width() const { return frame_.width; }
    int height() const { return frame_.height; }

    /* 논리 사각형을 네이티브 행 단위로 채운다. */
    void fill_rect(int x, int y, int w, int h, uint32_t color)
    {
        if (w <= 0 || h <= 0) return;
        const int x0 = std::max(0, x);
        const int y0 = std::max(0, y);
        const int x1 = std::min(frame_.width, x + w);
        const int y1 = std::min(frame_.height, y + h);
        if (x0 >= x1 || y0 >= y1) return;

        const int native_x0 = frame_.rotated ? frame_.height - y1 : x0;
        const int native_y0 = frame_.rotated ? x0 : y0;
        const int native_x1 = frame_.rotated ? frame_.height - y0 : x1;
        const int native_y1 = frame_.rotated ? x1 : y1;
        for (int row = native_y0; row < native_y1; ++row) {
            uint32_t *pixels = frame_.mat.ptr<uint32_t>(row);
            std::fill(pixels + native_x0, pixels + native_x1, color);
        }
    }

    /* 글리프 행의 연속 픽셀 구간(컴파일 타임 표)을 사각형 하나씩 채운다. */
    void text_left(int x, int y, const std::string &raw_text, int scale,
                   uint32_t color, int max_width = 0)
    {
        if (scale <= 0) return;
        const std::string text = clip_text(raw_text, scale, max_width);
        int cursor_x = x;
        for (char raw : text) {
            char c = raw;
            if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
            const GlyphRuns &runs = glyph_runs(c);
            for (int row = 0; row < kGlyphH; ++row) {
                for (const GlyphRun &run : runs[row]) {
                    if (run.length == 0) break;
                    fill_rect(cursor_x + run.start * scale, y + row * scale,
                              run.length * scale, scale, color);
                }
            }
            cursor_x += kGlyphAdvance * scale;
        }
    }

    void hud_text_left(int x, int y, const std::string &text, int scale,
                       uint32_t color, int max_width = 0)
    {
        text_left(x + kShadowOffset, y + kShadowOffset, text, scale, kShadow, max_width);
        text_left(x, y, text, scale, color, max_width);
    }

    void hud_text_center(int center_x, int y, const std::string &raw_text, int scale,
                         uint32_t color, int max_width = 0)
    {
        const std::string text = clip_text(raw_text, scale, max_width);
        const int x = center_x - text_width(text, scale) / 2;
        text_left(x + kShadowOffset, y + kShadowOffset, text, scale, kShadow);
        text_left(x, y, text, scale, color);
    }

    /* 반투명 상자: 밝은 테두리와 왼쪽 강조 띠. */
    void box(int x, int y, int w, int h, uint32_t accent)
    {
        fill_rect(x, y, w, h, kPanelFill);
        fill_rect(x, y, w, 1, kPanelEdgeTop);
        fill_rect(x, y + h - 1, w, 1, kPanelEdgeBottom);
        fill_rect(x, y, 1, h, kPanelEdgeLeft);
        fill_rect(x + w - 1, y, 1, h, kPanelEdgeRight);
        fill_rect(x, y, kPanelAccentW, h, accent);
    }

    /* 격자 패널 한 칸: 제목줄과 그 아래 구분선. */
    void titled_panel(int x, int y, uint32_t accent, const char *title)
    {
        box(x, y, kBoxW, kPanelH, accent);
        fill_rect(x + kPanelRuleInset, y + kPanelRuleY, kBoxW - 2 * kPanelRuleInset, 1, kPanelRule);
        hud_text_left(x + kBoxPadding, y + kPanelTitleY, title, 1, kDim, kColW);
    }

    /* 제목줄 오른쪽 끝의 작은 배지. */
    void title_badge(int box_x, int y, const std::string &text, uint32_t color)
    {
        hud_text_center(box_x + kBoxW - kBadgeCenterInset, y + kPanelTitleY, text, 1, color,
                        kBadgeMaxW);
    }

    void separator(int x, int y, int h)
    {
        fill_rect(x, y, 1, h, kSeparator);
    }

    /* 열 제목(scale 1) 위 값(scale 2)을 columns 열로. */
    void metric_columns(int box_x, int y, const char *const *labels,
                        const std::string *values, int columns, uint32_t color)
    {
        const int column_w = kMetricInnerW / columns;
        const int max_width = column_w - kMetricTextMargin;
        for (int column = 1; column < columns; ++column)
            separator(box_x + kMetricInnerMargin + column * column_w, y + kPanelRow1Y,
                      kMetricSeparatorH);
        for (int col = 0; col < columns; ++col) {
            const int center_x = box_x + kMetricInnerMargin +
                                 col * column_w + column_w / 2;
            hud_text_center(center_x, y + kPanelRow1Y, labels[col], 1, kDim, max_width);
            hud_text_center(center_x, y + kMetricValueY, values[col], 2, color, max_width);
        }
    }

private:
    static int text_width(const std::string &text, int scale)
    {
        return text.empty()
            ? 0 : static_cast<int>(text.size()) * kGlyphAdvance * scale - scale;
    }

    static std::string clip_text(const std::string &text, int scale, int max_width)
    {
        if (max_width <= 0 || text_width(text, scale) <= max_width) return text;
        std::string clipped = text;
        while (!clipped.empty() && text_width(clipped, scale) > max_width)
            clipped.pop_back();
        return clipped;
    }

    const Frame &frame_;
};

int right_box_x(const BitmapHud &ui)
{
    return ui.width() - kBoxW - kBoxMargin;
}

/* ---- 패널 ---- */

void draw_status_bar(BitmapHud &ui, const OverlayHudState &hud)
{
    ui.fill_rect(0, 0, ui.width(), kStatusBarH, status_color(hud));
}

void draw_speed(BitmapHud &ui, const OverlayHudState &hud)
{
    ui.hud_text_center(ui.width() / 2, kSpeedY,
                       format_text("%.0F", std::max(0.0f, hud.cluster_speed_kph)),
                       kSpeedScale, kWhite, kSpeedMaxW);
    ui.hud_text_center(ui.width() / 2, kSpeedUnitY, "KPH", 2, kDim);
}

void draw_openpilot_panel(BitmapHud &ui, const OverlayHudState &hud)
{
    const int x = kLeftBoxX + kBoxPadding;
    ui.titled_panel(kLeftBoxX, kPanelY0, status_color(hud), "OPENPILOT");
    ui.hud_text_left(x, kPanelY0 + kPanelRow1Y, op_status_text(hud), 3, control_color(hud));
    ui.hud_text_left(x, kPanelY0 + kPanelLine3Y,
                     format_text("PANDA %s  CAR %s",
                                 hud.panda_connected && hud.panda_healthy ? "OK" : "--",
                                 hud.vehicle_fresh ? "OK" : "--"),
                     1, panda_color(hud));
}

void draw_control_panel(BitmapHud &ui, const OverlayHudState &hud)
{
    const uint32_t color = control_color(hud);
    ui.titled_panel(kLeftBoxX, kPanelY1, color, "CONTROL");
    const char *labels[] = {"ANGLE", "DES", "APPLY", "DRIVER"};
    const std::string values[] = {
        format_text("%.0F", hud.steering_angle_deg),
        format_text("%d", hud.desired_torque),
        format_text("%d", hud.apply_torque),
        format_text("%d", hud.driver_torque),
    };
    ui.metric_columns(kLeftBoxX, kPanelY1, labels, values, 4, color);
}

void draw_system_panel(BitmapHud &ui, const OverlayHudState &hud)
{
    const int box_x = right_box_x(ui);
    const int x = box_x + kBoxPadding;
    ui.titled_panel(box_x, kPanelY0, system_color(hud), "SYSTEM");
    ui.hud_text_left(x, kPanelY0 + kPanelRow1Y,
                     format_text("AI %.1F FPS  CAM %.1F FPS", hud.model_fps, hud.preview_fps),
                     1, kDim, kColW);
    ui.hud_text_left(x, kPanelY0 + kPanelLine2Y,
                     format_text("HUD %.1F FPS", hud.overlay_fps), 1, kDim);
    ui.hud_text_left(x, kPanelY0 + kPanelLine3Y, network_text(hud), 1, network_color(hud), kColW);
}

void draw_health_panel(BitmapHud &ui, const OverlayHudState &hud)
{
    const uint32_t color = health_color(hud);
    const int box_x = right_box_x(ui);
    ui.titled_panel(box_x, kPanelY1, color, "HEALTH");
    const char *labels[] = {"CPU", "TEMP", "MEM", "DISK"};
    const std::string values[] = {
        format_text("%.0F%%", hud.cpu_percent),
        format_text("%.0FC", hud.cpu_temp_c),
        format_text("%.0F%%", hud.memory_percent),
        format_text("%.0F%%", hud.storage_percent),
    };
    ui.metric_columns(box_x, kPanelY1, labels, values, 4, color);
}

void draw_calibration_panel(BitmapHud &ui, const OverlayHudState &hud)
{
    const uint32_t color = calibration_color(hud);
    const int box_x = right_box_x(ui);
    ui.titled_panel(box_x, kPanelY2, color, "CALIBRATION");
    ui.title_badge(box_x, kPanelY2,
                   format_text("%s B%d", calibration_status_text(hud),
                               std::max(0, hud.calibration_valid_blocks)),
                   color);
    const char *labels[] = {"ROLL", "PITCH", "YAW"};
    const std::string values[] = {
        format_text("%.2F", hud.calibration_roll_deg),
        format_text("%.2F", hud.calibration_pitch_deg),
        format_text("%.2F", hud.calibration_yaw_deg),
    };
    ui.metric_columns(box_x, kPanelY2, labels, values, 3, color);
}

void draw_drive_panel(BitmapHud &ui, const OverlayHudState &hud)
{
    const uint32_t color = control_color(hud);
    const int x = kLeftBoxX + kBoxPadding;
    ui.titled_panel(kLeftBoxX, kPanelY2, color, "DRIVE");
    ui.title_badge(kLeftBoxX, kPanelY2, lateral_mode_text(hud), lateral_mode_color(hud));
    ui.hud_text_left(x, kPanelY2 + kPanelRow1Y, cruise_text(hud), 2, color, kColW);
    ui.hud_text_left(x, kPanelY2 + kPanelRow2Y,
                     format_text("GEAR %s  CRZ %s  %s", gear_text(hud.gear),
                                 hud.cruise_active ? "ON" : "OFF",
                                 active_block_text(hud).c_str()),
                     1, color, kColW);
}

void draw_lead_panel(BitmapHud &ui, const LeadInfo &lead)
{
    const uint32_t color = lead_color(lead);
    const int box_x = right_box_x(ui);
    const int x = box_x + kBoxPadding;
    ui.titled_panel(box_x, kPanelY3, color, "LEAD");
    if (lead.any()) {
        ui.hud_text_left(x, kPanelY3 + kPanelRow1Y,
                         format_text("DIST %.0FM P %.0F%%", lead.distance_m,
                                     lead.vision ? lead.probability * 100.0f : 0.0f),
                         2, color, kColW);
        ui.hud_text_left(x, kPanelY3 + kPanelRow2Y,
                         format_text("REL %+.0F KPH", lead.relative_speed_kph), 2, color, kColW);
    } else {
        ui.hud_text_left(x, kPanelY3 + kPanelRow1Y, "NO LEAD", 2, kDim);
        ui.hud_text_left(x, kPanelY3 + kPanelRow2Y, "REL -- KPH", 2, kDim);
    }
}

void draw_auto_hold(BitmapHud &ui, const OverlayHudState &hud)
{
    if (!hud.brake_hold) return;
    const int x = (ui.width() - kAutoHoldW) / 2;
    ui.box(x, kAutoHoldY, kAutoHoldW, kAutoHoldH, kGreen);
    ui.hud_text_center(ui.width() / 2, kAutoHoldY + kAutoHoldTextY, "AUTO HOLD", 3, kGreen);
}

void draw_tpms_panel(BitmapHud &ui, const OverlayHudState &hud)
{
    const TpmsRange range = tpms_range(hud);
    const uint32_t color = tpms_color(hud, range);
    const int x = kLeftBoxX + kBoxPadding;
    ui.titled_panel(kLeftBoxX, kPanelY3, color, "TPMS");
    ui.title_badge(kLeftBoxX, kPanelY3, range.bar ? "BAR" : "PSI", color);
    ui.separator(kLeftBoxX + kMetricInnerMargin + kMetricInnerW / 2, kPanelY3 + kPanelRow1Y,
                 kMetricSeparatorH);

    struct Wheel {
        const char *name;
        float pressure;
        int column_x;
        int row_y;
    };
    const Wheel wheels[] = {
        {"FL", hud.tpms_pressure_fl, x, kPanelY3 + kPanelRow1Y},
        {"FR", hud.tpms_pressure_fr, x + kTpmsColumnOffset, kPanelY3 + kPanelRow1Y},
        {"RL", hud.tpms_pressure_rl, x, kPanelY3 + kPanelRow2Y},
        {"RR", hud.tpms_pressure_rr, x + kTpmsColumnOffset, kPanelY3 + kPanelRow2Y},
    };
    for (const Wheel &wheel : wheels) {
        ui.hud_text_left(wheel.column_x, wheel.row_y,
                         pressure_text(hud, range, wheel.name, wheel.pressure), 2,
                         pressure_color(hud, range, wheel.pressure), kTpmsColumnW);
    }
}

void draw_alert(BitmapHud &ui, const OverlayHudState &hud, const ParsedModelOutput &output)
{
    const Alert alert = select_alert(hud);
    if (alert.empty()) return;

    const int alert_x = (ui.width() - kAlertW) / 2;
    const int alert_y = ui.height() - kAlertBottomMargin;
    ui.box(alert_x, alert_y, kAlertW, kAlertH, alert.color);
    ui.hud_text_center(ui.width() / 2, alert_y + kAlertTitleY, alert.title, 3, alert.color,
                       kAlertW - kBoxTextInset);
    ui.hud_text_center(ui.width() / 2, alert_y + kAlertDetailY,
                       alert.departure
                           ? "CHECK ROAD AND PROCEED"
                           : format_text("M%s CTL%s PND%s",
                                         output.valid ? "OK" : "--",
                                         hud.vehicle_fresh ? "OK" : "--",
                                         hud.panda_connected ? "OK" : "--"),
                       1, kWhite, kAlertW - kBoxTextInset);
}

void draw_traffic_signal(const Frame &frame, const OverlayHudState &hud,
                         const TrafficSignalSprites *sprites)
{
    const bool green = hud.departure_alert_type == DepartureAlertType::green_light;
    const bool red = hud.green_light_alert_armed;
    if ((!green && !red) || sprites == nullptr) return;

    const int center_x = frame.width - kSignalRightOffset;
    blit_traffic_signal(frame, green ? sprites->green : sprites->red,
                        center_x - kSignalHousingCenterX, kSignalCenterY - kSignalHousingCenterY);
}

/* 덮어쓰기 순서: 신호등 스프라이트는 LEAD 패널 위에, 알림 상자는 그 위에. */
void draw_hud(const Frame &frame, const OverlayHudState &hud,
              const ParsedModelOutput &output, const LeadInfo &lead,
              const TrafficSignalSprites *sprites)
{
    BitmapHud ui(frame);
    draw_status_bar(ui, hud);
    draw_speed(ui, hud);
    draw_openpilot_panel(ui, hud);
    draw_control_panel(ui, hud);
    draw_system_panel(ui, hud);
    draw_health_panel(ui, hud);
    draw_calibration_panel(ui, hud);
    draw_drive_panel(ui, hud);
    draw_lead_panel(ui, lead);
    draw_traffic_signal(frame, hud, sprites);
    draw_auto_hold(ui, hud);
    draw_tpms_panel(ui, hud);
    draw_alert(ui, hud, output);
}

/* ---- 장면(모델 출력) ---- */

/* 모델 좌표는 180° 뒤집힌 화면 기준이라 투영 후 뒤집는다. 논리 좌표를 돌려준다. */
std::optional<cv::Point> project_display_point(const Frame &frame,
                                               const ProjectionState &projection,
                                               float x, float y, float z)
{
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return std::nullopt;
    int px = 0;
    int py = 0;
    if (!project_point(projection, x, y, z, frame.width, frame.height, &px, &py))
        return std::nullopt;
    return cv::Point(frame.width - 1 - px, frame.height - 1 - py);
}

double quad_area(const cv::Point &a, const cv::Point &b, const cv::Point &c, const cv::Point &d)
{
    const cv::Point quad[] = {a, b, c, d};
    double area = 0.0;
    for (int i = 0; i < 4; ++i) {
        const cv::Point &p = quad[i];
        const cv::Point &q = quad[(i + 1) % 4];
        area += static_cast<double>(p.x) * q.y - static_cast<double>(q.x) * p.y;
    }
    return area * 0.5;
}

/* 궤적 양쪽을 투영해 띠 다각형으로 채운다. 접히거나 뒤로 가는 구간에서 멈춘다. */
void draw_model_ribbon(const Frame &frame,
                       const std::array<ModelPoint, kTrajectorySize> &points,
                       float half_width, float z_offset, float max_distance,
                       const cv::Scalar &color, const ProjectionState &projection)
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

        const auto left = project_display_point(frame, projection, point.x,
                                                point.y - half_width, point.z + z_offset);
        const auto right = project_display_point(frame, projection, point.x,
                                                 point.y + half_width, point.z + z_offset);
        if (!left || !right) {
            if (!pairs.empty()) break;
            continue;
        }

        if (!pairs.empty()) {
            const ProjectedPair &previous = pairs.back();
            const double area = quad_area(previous.left, *left, *right, previous.right);
            if (std::abs(area) < 0.5 ||
                (polygon_winding != 0.0 && area * polygon_winding <= 0.0)) {
                break;
            }
            if (polygon_winding == 0.0) polygon_winding = area;
        }

        pairs.push_back({*left, *right});
        previous_x = point.x;
    }

    if (pairs.size() < 2) return;

    std::vector<cv::Point> vertices;
    vertices.reserve(pairs.size() * 2);
    for (const ProjectedPair &pair : pairs) vertices.push_back(frame.native(pair.left.x, pair.left.y));
    for (auto it = pairs.rbegin(); it != pairs.rend(); ++it)
        vertices.push_back(frame.native(it->right.x, it->right.y));

    const cv::Point *polygon[] = {vertices.data()};
    const int count[] = {static_cast<int>(vertices.size())};
    cv::fillPoly(frame.mat, polygon, count, 1, color, cv::LINE_8);
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

cv::Scalar road_edge_color(float confidence)
{
    return bgra(60, 60, 255, static_cast<int>(confidence * 200.0f));
}

/* 리드 위치에 세 겹 삼각형: 그림자, 테두리, 위험도에 따라 초록→빨강인 안쪽. */
void draw_lead_marker(const Frame &frame, const LeadInfo &lead, const OverlayHudState &hud,
                      const ProjectionState &projection)
{
    const auto projected = project_display_point(frame, projection, lead.point.x,
                                                 lead.point.y, kModelHeight);
    if (!projected) return;

    const int size = std::clamp(
        static_cast<int>(kLeadMarkerSizeBase - lead.distance_m * kLeadMarkerSizePerMeter),
        kLeadMarkerSizeMin, kLeadMarkerSizeMax);
    const int outer_half_width = size * 5 / 4;
    const int cx = std::clamp(projected->x, outer_half_width + 4,
                              frame.width - outer_half_width - 4);
    const int cy = std::clamp(projected->y + size + kLeadMarkerOffsetY, size + 4,
                              frame.height - size - 4);

    const float relative_speed_mps = lead.point.velocity - hud.ego_speed_kph / 3.6f;
    const float distance_risk = std::clamp(1.0f - lead.distance_m / kLeadRiskDistanceM, 0.0f, 1.0f);
    const float closing_risk = std::clamp(-relative_speed_mps / kLeadRiskClosingMps, 0.0f, 1.0f);
    const float risk = std::clamp(distance_risk + closing_risk, 0.0f, 1.0f);
    const int confidence_alpha = static_cast<int>(
        170.0f + 85.0f * std::clamp(lead.probability, 0.0f, 1.0f));
    const int inner_green = static_cast<int>(165.0f - 125.0f * risk);

    auto fill_triangle = [&](int center_y, int half_width, int half_height,
                             const cv::Scalar &color) {
        const cv::Point vertices[] = {
            frame.native(cx, center_y - half_height),
            frame.native(cx - half_width, center_y + half_height),
            frame.native(cx + half_width, center_y + half_height),
        };
        cv::fillConvexPoly(frame.mat, vertices, 3, color, cv::LINE_8);
    };

    fill_triangle(cy + 2, outer_half_width + 3, size + 3, bgra(0, 0, 0, 130));
    fill_triangle(cy, outer_half_width, size, bgra(35, 220, 255, confidence_alpha));
    fill_triangle(cy - 1, std::max(4, outer_half_width - 4), std::max(4, size - 4),
                  bgra(35, inner_green, 255, confidence_alpha));
}

void draw_scene(const Frame &frame, const ParsedModelOutput &output,
                const ProjectionState &projection, const OverlayHudState &hud,
                const LeadInfo &lead)
{
    if (!output.valid) return;

    const float max_distance = output.plan.valid
        ? std::clamp(output.plan.points.back().x, kMinDrawDistance, kMaxDrawDistance)
        : kMaxDrawDistance;

    if (output.plan.valid) {
        draw_model_ribbon(frame, output.plan.points, kPathHalfWidth, kModelHeight,
                          max_distance, openpilot_path_color(hud), projection);
    }

    if (!hud.laneless_mode) {
        for (const ParsedLaneLine &lane : output.lanes) {
            if (!lane.valid || lane.probability < kLaneMinProbability) continue;
            draw_model_ribbon(frame, lane.points,
                              std::max(kLaneHalfWidthMin,
                                       kLaneHalfWidthPerProbability * lane.probability),
                              0.0f, max_distance, openpilot_lane_color(lane.probability),
                              projection);
        }

        for (const ParsedRoadEdge &edge : output.road_edges) {
            if (!edge.valid) continue;
            const float confidence = std::clamp(1.0f - edge.std, 0.0f, 1.0f);
            if (confidence < kEdgeMinConfidence) continue;
            draw_model_ribbon(frame, edge.points, kEdgeHalfWidth, 0.0f, max_distance,
                              road_edge_color(confidence), projection);
        }
    }

    if (lead.vision) draw_lead_marker(frame, lead, hud, projection);
}

/* ---- 깜빡이 ---- */

void draw_turn_chevron(const Frame &frame, int inner_x, bool points_left, int alpha)
{
    const int direction = points_left ? -1 : 1;
    const int shoulder_x = inner_x + direction * (kTurnChevronW / 2);
    const int tip_x = inner_x + direction * kTurnChevronW;

    const cv::Point vertices[] = {
        frame.native(inner_x, kTurnCenterY - kTurnChevronHalfH),
        frame.native(shoulder_x, kTurnCenterY - kTurnChevronHalfH),
        frame.native(tip_x, kTurnCenterY),
        frame.native(shoulder_x, kTurnCenterY + kTurnChevronHalfH),
        frame.native(inner_x, kTurnCenterY + kTurnChevronHalfH),
        frame.native(shoulder_x, kTurnCenterY),
    };
    const cv::Point *points = vertices;
    const int point_count = 6;
    cv::fillPoly(frame.mat, &points, &point_count, 1, bgra(70, 230, 255, alpha), cv::LINE_8);
}

void draw_turn_signals(const Frame &frame, const OverlayHudState &hud)
{
    if (!hud.left_blinker && !hud.right_blinker) return;
    const int step = std::clamp(hud.turn_signal_step, 0, kTurnSignalSteps - 1);
    if (step >= kTurnLitSteps) return;

    const int center_x = frame.width / 2;
    auto draw_side = [&](bool active, bool points_left) {
        if (!active) return;
        const int direction = points_left ? -1 : 1;
        const int first_inner_x = center_x + direction * kTurnInnerOffset;
        for (int i = 0; i < 3; ++i) {
            if (step < kTurnChevronStartStep[i]) break;
            draw_turn_chevron(frame, first_inner_x + direction * i * kTurnChevronStep,
                              points_left, kTurnChevronAlpha[i]);
        }
    };

    draw_side(hud.left_blinker, true);
    draw_side(hud.right_blinker, false);
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

/* ---- 공유 상태 → HUD 상태 ---- */

void hud_apply_panda_state(const K230PandaState &panda, bool fresh, OverlayHudState *hud)
{
    hud->panda_connected = fresh && panda.connected != 0;
    hud->panda_healthy = fresh && panda.comms_healthy != 0;
    hud->panda_faults = fresh ? panda.faults : 0;
}

void hud_apply_control_state(const K230ControlState &c, bool fresh, OverlayHudState *hud)
{
    hud->controller_enabled = fresh && c.enabled != 0;
    hud->controller_engaged = fresh && c.engaged != 0;
    hud->controller_active = fresh && c.active != 0;
    hud->lateral_mode_available = fresh;
    hud->laneless_mode = fresh && (c.hud_flags & kK230HudFlagLaneless) != 0;
    hud->vehicle_fresh = fresh && c.vehicle_fresh != 0;
    hud->steering_fault = fresh && c.steering_fault != 0;
    hud->left_blinker = fresh && c.left_blinker != 0;
    hud->right_blinker = fresh && c.right_blinker != 0;
    hud->cruise_active = fresh && c.cruise_active != 0;
    hud->brake_hold = fresh && (c.hud_flags & kK230HudFlagBrakeHold) != 0;
    hud->gear = fresh ? c.gear : 0;
    hud->cluster_speed_kph = fresh ? c.cluster_speed_kph : 0.0f;
    hud->ego_speed_kph = fresh ? c.ego_speed_kph : 0.0f;
    hud->cruise_max_speed_kph = fresh ? c.cruise_max_speed_kph : 0.0f;
    hud->cruise_command_speed_kph = fresh ? c.cruise_command_speed_kph : 0.0f;
    hud->radar_lead_valid = fresh && c.radar_lead_valid != 0;
    hud->radar_lead_distance_m = fresh ? c.radar_lead_distance_m : 0.0f;
    hud->radar_lead_relative_speed_mps = fresh ? c.radar_lead_relative_speed_mps : 0.0f;
    hud->departure_alert_type = fresh
        ? static_cast<DepartureAlertType>(c.departure_alert_type)
        : DepartureAlertType::none;
    hud->green_light_alert_armed = fresh && c.green_light_alert_armed != 0;
    hud->tpms_valid = fresh && c.tpms_valid != 0;
    hud->tpms_unit = fresh ? static_cast<int>(c.tpms_unit) : 0;
    hud->tpms_pressure_fl = fresh ? c.tpms_pressure_fl : 0.0f;
    hud->tpms_pressure_fr = fresh ? c.tpms_pressure_fr : 0.0f;
    hud->tpms_pressure_rl = fresh ? c.tpms_pressure_rl : 0.0f;
    hud->tpms_pressure_rr = fresh ? c.tpms_pressure_rr : 0.0f;
    hud->tpms_warning = fresh && c.tpms_warning != 0;
    hud->steering_angle_deg = fresh ? c.steering_angle_deg : 0.0f;
    hud->normalized_output = fresh ? c.normalized_output : 0.0f;
    hud->desired_torque = fresh ? c.desired_torque : 0;
    hud->apply_torque = fresh ? c.apply_torque : 0;
    hud->driver_torque = fresh ? c.driver_torque : 0;
    std::snprintf(hud->active_block, sizeof(hud->active_block), "%s",
                  fresh ? c.active_block : "control_stale");
}

void hud_apply_model_state(const K230ModelState &model, bool fresh, OverlayHudState *hud)
{
    const K230CalibrationState &calibration = model.calibration;
    hud->calibration_available = fresh;
    hud->calibration_status = calibration.status;
    hud->calibration_valid_blocks = calibration.valid_blocks;
    hud->calibration_roll_deg = rad_to_deg(calibration.roll);
    hud->calibration_pitch_deg = rad_to_deg(calibration.pitch);
    hud->calibration_yaw_deg = rad_to_deg(calibration.yaw);
}

void hud_apply_manager_state(const K230ManagerState &manager, bool fresh, bool model_ok,
                             OverlayHudState *hud)
{
    const unsigned total = fresh
        ? std::min<unsigned>(manager.process_count, kK230MaxProcesses) : 0;
    unsigned running = 0;
    for (unsigned i = 0; i < total; ++i) running += manager.processes[i].running ? 1U : 0U;
    hud->services_healthy = fresh && total >= 3 && running == total && model_ok;
}

/* ---- 렌더러 ---- */

bool OverlayRenderer::load_assets(const std::string &dir)
{
    auto sprites = std::make_shared<TrafficSignalSprites>();
    const bool ready =
        sprites->red.load(dir + "/traffic_wait_red_retro-270x155-v3.png") &&
        sprites->green.load(dir + "/traffic_go_green_retro-270x155-v3.png");
    if (!ready) {
        std::fprintf(stderr, "overlay: traffic signal PNG assets unavailable in %s\n",
                     dir.c_str());
        sprites_.reset();
        return false;
    }
    sprites_ = std::move(sprites);
    return true;
}

void OverlayRenderer::draw(const OverlayTarget &target, const ParsedModelOutput &output,
                           const ProjectionState &projection,
                           const OverlayHudState &hud,
                           bool rotate_landscape) const
{
    const int width = static_cast<int>(target.width);
    const int height = static_cast<int>(target.height);
    cv::Mat mat(height, width, CV_8UC4, target.map, static_cast<size_t>(target.stride));
    mat.setTo(cv::Scalar(0, 0, 0, 0));
    const Frame frame{mat, rotate_landscape ? height : width,
                      rotate_landscape ? width : height, rotate_landscape};

    const LeadInfo lead = lead_info(hud, output);
    draw_scene(frame, output, projection, hud, lead);
    draw_turn_signals(frame, hud);
    draw_hud(frame, hud, output, lead, sprites_.get());
}
