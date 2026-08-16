#pragma once

/* LD 차선(이미지 좌표)을 평면 가정으로 도로 좌표에 역투영해 supercombo와
 * 같은 프레임에서 차로 중앙 오프셋을 만든다. 섀도우 모드: 제어에는 쓰지
 * 않고 관측치만 계산한다. 오프라인 검증(2026-08-28, 야간 시내 30초):
 * 10m 지점 중앙 오프셋이 supercombo와 corr 0.94, |err| 6cm. */

#include <vector>

#include "lane_decode.h"
#include "model_output.h"

struct LdFusionResult {
    bool valid = false;
    float ld_center = 0.0f;   // +좌측 [m]
    float sc_center = 0.0f;   // +좌측 [m]
    float trim = 0.0f;        // ld_center - sc_center
    float ld_width = 0.0f;
};

/* rpy: 라이브 캘리브레이션(rad). src_w/h: LD 입력 소스 해상도.
 * dist_m 지점에서 host 좌/우 차선으로 중앙을 계산한다. */
LdFusionResult ld_fusion_compute(const std::vector<DecodedLine> &ld_lines,
                                 const ParsedModelOutput &sc,
                                 const float rpy[3],
                                 unsigned src_w, unsigned src_h,
                                 float camera_height,
                                 float dist_m = 10.0f,
                                 float min_point_conf = 0.5f);

/* supercombo가 레인리스로 떨어질 상황(host 차선 확률 낮음)에서 LD host
 * 차선이 게이트를 통과하면 SC lanes[1]/[2]를 LD 투영 폴리라인으로 교체하고
 * 확률을 승격시킨다. 반환: 승격 수행 여부. gates_ok에는 (승격 여부와
 * 무관하게) LD 게이트 통과 여부를 돌려줘 호출자가 연속성 카운트에 쓴다. */
struct LdPromotionParams {
    float sc_prob_below = 0.35f;  // 이보다 낮을 때만 개입 (lane 모드 문턱 0.3/0.4 대비 여유)
    float promoted_prob = 0.7f;
    float promoted_std = 0.3f;
    float min_point_conf = 0.5f;
};

bool ld_promote_lanes(const std::vector<DecodedLine> &ld_lines,
                      const float rpy[3],
                      unsigned src_w, unsigned src_h,
                      float camera_height,
                      ParsedModelOutput *sc,
                      bool *gates_ok,
                      const LdPromotionParams &params = {});
