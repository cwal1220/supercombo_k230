#include "ld_fusion.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "app_config.h"
#include "projection.h"

namespace {

/* view_from_calib = [[0,-1,0],[0,0,-1],[1,0,0]] * R(rpy). 직교행렬이라
 * 역행렬은 전치. projection.cc의 디스플레이용 축 반전은 섞지 않는다. */
void calib_from_view(const float rpy[3], float *m)
{
    float rot[9];
    rotation_from_rpy(rpy[0], rpy[1], rpy[2], rot);
    const float view[9] = {
        -rot[3], -rot[4], -rot[5],
        -rot[6], -rot[7], -rot[8],
        rot[0], rot[1], rot[2],
    };
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            m[r * 3 + c] = view[c * 3 + r];
}

/* 한 라인의 dist_m 지점 횡위치(+좌측)를 역투영+선형보간으로 구한다. */
bool lane_y_at(const DecodedLine &line, const float *cfv,
               unsigned src_w, unsigned src_h, float camera_height,
               float fx, float fy, float cx, float cy,
               float dist_m, float min_point_conf, float *y_out)
{
    float prev_x = 0.0f;
    float prev_y = 0.0f;
    bool have_prev = false;
    for (const auto &pt : line.points) {
        if (!lane_point_valid(pt) || pt.confidence < min_point_conf) continue;
        const float u = pt.x * static_cast<float>(src_w);
        const float v = pt.y * static_cast<float>(src_h);
        const float dv[3] = {(u - cx) / fx, (v - cy) / fy, 1.0f};
        const float d0 = cfv[0] * dv[0] + cfv[1] * dv[1] + cfv[2] * dv[2];
        const float d1 = cfv[3] * dv[0] + cfv[4] * dv[1] + cfv[5] * dv[2];
        const float d2 = cfv[6] * dv[0] + cfv[7] * dv[1] + cfv[8] * dv[2];
        if (d2 >= -1e-6f) continue;
        const float t = -camera_height / d2;
        const float x = t * d0;
        const float y = t * d1;
        if (x < 1.0f || x > 120.0f) continue;
        /* 행 앵커는 이미지 아래(가까움)에서 위(멈)로 정렬돼 있어 x는 단조 증가. */
        if (have_prev && prev_x < dist_m && x >= dist_m) {
            const float w = (dist_m - prev_x) / (x - prev_x);
            *y_out = prev_y + w * (y - prev_y);
            return true;
        }
        prev_x = x;
        prev_y = y;
        have_prev = true;
    }
    return false;
}

bool sc_lane_y_at(const ParsedLaneLine &lane, float dist_m, float *y_out)
{
    if (!lane.valid) return false;
    for (size_t i = 1; i < lane.points.size(); ++i) {
        const float x0 = lane.points[i - 1].x;
        const float x1 = lane.points[i].x;
        if (x0 < dist_m && x1 >= dist_m && x1 > x0) {
            const float w = (dist_m - x0) / (x1 - x0);
            /* supercombo 차선 y는 +우측 규약: +좌측으로 뒤집는다. */
            *y_out = -(lane.points[i - 1].y + w * (lane.points[i].y - lane.points[i - 1].y));
            return true;
        }
    }
    return false;
}

}  // namespace

LdFusionResult ld_fusion_compute(const std::vector<DecodedLine> &ld_lines,
                                 const ParsedModelOutput &sc,
                                 const float rpy[3],
                                 unsigned src_w, unsigned src_h,
                                 float camera_height,
                                 float dist_m, float min_point_conf)
{
    LdFusionResult out;
    static int debug_left = std::getenv("SUPERCOMBO_LD_DEBUG") ? 8 : 0;

    const DecodedLine *host_l = nullptr;
    const DecodedLine *host_r = nullptr;
    for (const auto &line : ld_lines) {
        if (line.kind != DecodedLineKind::Lane) continue;
        if (line.slot == 0) host_l = &line;
        if (line.slot == 1) host_r = &line;
    }
    if (!host_l || !host_r) {
        if (debug_left > 0) { --debug_left;
            std::fprintf(stderr, "\nldfus: host missing l=%d r=%d nlines=%zu\n",
                         host_l != nullptr, host_r != nullptr, ld_lines.size()); }
        return out;
    }

    float cfv[9];
    calib_from_view(rpy, cfv);
    const float fx = default_input_warp_fx(src_w);
    const float fy = default_input_warp_fy(src_h);
    const float cx = default_input_warp_cx(src_w);
    const float cy = default_input_warp_cy(src_h);

    float ld_l = 0.0f;
    float ld_r = 0.0f;
    if (!lane_y_at(*host_l, cfv, src_w, src_h, camera_height, fx, fy, cx, cy,
                   dist_m, min_point_conf, &ld_l) ||
        !lane_y_at(*host_r, cfv, src_w, src_h, camera_height, fx, fy, cx, cy,
                   dist_m, min_point_conf, &ld_r)) {
        if (debug_left > 0) { --debug_left;
            std::fprintf(stderr, "\nldfus: interp fail rpy=(%.3f %.3f %.3f) npts=%zu/%zu\n",
                         rpy[0], rpy[1], rpy[2],
                         host_l->points.size(), host_r->points.size()); }
        return out;
    }

    const float width = ld_l - ld_r;
    if (width < 2.4f || width > 4.5f) {
        if (debug_left > 0) { --debug_left;
            std::fprintf(stderr, "\nldfus: width gate %.2f (l=%.2f r=%.2f)\n", width, ld_l, ld_r); }
        return out;
    }

    float sc_l = 0.0f;
    float sc_r = 0.0f;
    if (!sc_lane_y_at(sc.lanes[1], dist_m, &sc_l) ||
        !sc_lane_y_at(sc.lanes[2], dist_m, &sc_r)) {
        if (debug_left > 0) { --debug_left;
            std::fprintf(stderr, "\nldfus: sc interp fail\n"); }
        return out;
    }

    out.valid = true;
    out.ld_center = 0.5f * (ld_l + ld_r);
    out.sc_center = 0.5f * (sc_l + sc_r);
    out.trim = out.ld_center - out.sc_center;
    out.ld_width = width;
    return out;
}

namespace {

/* host 한 슬롯을 도로 좌표 폴리라인(x 단조증가)으로 투영한다. */
bool project_polyline(const DecodedLine &line, const float *cfv,
                      unsigned src_w, unsigned src_h, float camera_height,
                      float fx, float fy, float cx, float cy, float min_point_conf,
                      std::vector<float> *xs, std::vector<float> *ys)
{
    xs->clear();
    ys->clear();
    for (const auto &pt : line.points) {
        if (!lane_point_valid(pt) || pt.confidence < min_point_conf) continue;
        const float u = pt.x * static_cast<float>(src_w);
        const float v = pt.y * static_cast<float>(src_h);
        const float dv[3] = {(u - cx) / fx, (v - cy) / fy, 1.0f};
        const float d0 = cfv[0] * dv[0] + cfv[1] * dv[1] + cfv[2] * dv[2];
        const float d1 = cfv[3] * dv[0] + cfv[4] * dv[1] + cfv[5] * dv[2];
        const float d2 = cfv[6] * dv[0] + cfv[7] * dv[1] + cfv[8] * dv[2];
        if (d2 >= -1e-6f) continue;
        const float t = -camera_height / d2;
        const float x = t * d0;
        const float y = t * d1;
        if (x < 1.0f || x > 120.0f) continue;
        if (!xs->empty() && x <= xs->back()) continue;
        xs->push_back(x);
        ys->push_back(y);
    }
    return xs->size() >= 6;
}

float polyline_y_at(const std::vector<float> &xs, const std::vector<float> &ys, float x)
{
    if (x <= xs.front()) return ys.front();
    if (x >= xs.back()) {
        /* 마지막 구간 기울기로 직선 외삽 (저속 지평선 내에서만 의미) */
        const size_t n = xs.size();
        const float slope = (ys[n - 1] - ys[n - 2]) / (xs[n - 1] - xs[n - 2]);
        return ys[n - 1] + slope * (x - xs[n - 1]);
    }
    for (size_t i = 1; i < xs.size(); ++i) {
        if (xs[i] >= x) {
            const float w = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
            return ys[i - 1] + w * (ys[i] - ys[i - 1]);
        }
    }
    return ys.back();
}

}  // namespace

bool ld_promote_lanes(const std::vector<DecodedLine> &ld_lines,
                      const float rpy[3],
                      unsigned src_w, unsigned src_h,
                      float camera_height,
                      ParsedModelOutput *sc,
                      bool *gates_ok,
                      const LdPromotionParams &params)
{
    if (gates_ok) *gates_ok = false;

    const DecodedLine *host_l = nullptr;
    const DecodedLine *host_r = nullptr;
    for (const auto &line : ld_lines) {
        if (line.kind != DecodedLineKind::Lane) continue;
        if (line.slot == 0) host_l = &line;
        if (line.slot == 1) host_r = &line;
    }
    if (!host_l || !host_r) return false;

    float cfv[9];
    calib_from_view(rpy, cfv);
    const float fx = default_input_warp_fx(src_w);
    const float fy = default_input_warp_fy(src_h);
    const float cx = default_input_warp_cx(src_w);
    const float cy = default_input_warp_cy(src_h);

    std::vector<float> lx, ly, rx, ry;
    if (!project_polyline(*host_l, cfv, src_w, src_h, camera_height, fx, fy, cx, cy,
                          params.min_point_conf, &lx, &ly) ||
        !project_polyline(*host_r, cfv, src_w, src_h, camera_height, fx, fy, cx, cy,
                          params.min_point_conf, &rx, &ry))
        return false;

    /* 10m 차폭 검사 = 캘리브레이션/오검출 게이트 */
    const float width = polyline_y_at(lx, ly, 10.0f) - polyline_y_at(rx, ry, 10.0f);
    if (width < 2.4f || width > 4.5f) return false;
    /* LD 커버리지가 최소 15m는 되어야 경로 재료로 쓸 수 있다. */
    if (std::min(lx.back(), rx.back()) < 15.0f) return false;
    if (gates_ok) *gates_ok = true;

    const float sc_prob = std::max(sc->lanes[1].probability, sc->lanes[2].probability);
    if (sc_prob >= params.sc_prob_below) return false;

    struct Patch { int idx; std::vector<float> *xs; std::vector<float> *ys; };
    const Patch patches[] = {{1, &lx, &ly}, {2, &rx, &ry}};
    for (const auto &patch : patches) {
        ParsedLaneLine &lane = sc->lanes[patch.idx];
        for (auto &point : lane.points) {
            /* SC 차선 y는 +우측 규약이라 부호를 뒤집어 넣는다. */
            point.y = -polyline_y_at(*patch.xs, *patch.ys, point.x);
        }
        lane.valid = true;
        lane.probability = params.promoted_prob;
        lane.std = params.promoted_std;
    }
    return true;
}
