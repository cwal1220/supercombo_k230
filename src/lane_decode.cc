#include "lane_decode.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>



namespace {

constexpr float kRoiX1 = 0.0f;
constexpr float kRoiX2 = 1.0f;
constexpr float kLaneInnerEdgeMaxOffsetPx = 24.0f;
constexpr int kMaxConnectedRowGap = 4;
constexpr int kOutputCount = 15;
constexpr int kLaneSlots = 6;
constexpr int kBoundarySlots = 4;
constexpr int kRowsPerLine = 72;
constexpr int kLaneLogitsPerSlot = 12;
constexpr int kBoundaryLogitsPerSlot = 10;
constexpr int kLaneDeltaSize = 144;
constexpr int kBoundaryDeltaSize = 72;
constexpr int kLaneDeltaWidthOffset = 72;
constexpr int kLaneLogitsOutput = 1;
constexpr int kBoundaryLogitsOutput = 2;
constexpr int kAnchorOutput = 3;
constexpr int kPointConfidenceOutput = 4;
constexpr int kLaneDeltaOutputStart = 5;
constexpr int kBoundaryDeltaOutputStart = 11;

struct DecodeRect {
  float x1 = 0.0f;
  float y1 = 0.0f;
  float x2 = 1.0f;
  float y2 = 1.0f;
};

constexpr std::array<float, kRowsPerLine> kPriorYs = {
  1.00000000f, 0.98119122f, 0.95924765f, 0.94043887f, 0.91849530f, 0.89968652f,
  0.88087773f, 0.86206895f, 0.84326017f, 0.82445139f, 0.80877745f, 0.78996867f,
  0.77429467f, 0.75862068f, 0.74294674f, 0.72727275f, 0.71159875f, 0.69592476f,
  0.68025076f, 0.66457683f, 0.65203762f, 0.63636363f, 0.62068969f, 0.60815048f,
  0.59561127f, 0.57993734f, 0.56739813f, 0.55485892f, 0.54231977f, 0.52978057f,
  0.51724142f, 0.50470221f, 0.49216300f, 0.47962382f, 0.47021943f, 0.45768023f,
  0.44514105f, 0.43573666f, 0.42319748f, 0.41379309f, 0.40125391f, 0.39184952f,
  0.38557991f, 0.37617555f, 0.36677116f, 0.36050156f, 0.35109717f, 0.34169278f,
  0.33542320f, 0.32601881f, 0.31974921f, 0.31034482f, 0.30094042f, 0.29467085f,
  0.28526646f, 0.27586207f, 0.26959246f, 0.26018807f, 0.25078368f, 0.24451411f,
  0.23510972f, 0.22570533f, 0.21943574f, 0.21003135f, 0.20062695f, 0.17241380f,
  0.14420062f, 0.11598746f, 0.08777429f, 0.05956113f, 0.03134796f, 0.00000000f,
};

float clampf(float v, float lo, float hi) {
  return std::min(std::max(v, lo), hi);
}

int clampi(int v, int lo, int hi) {
  return std::min(std::max(v, lo), hi);
}

float sigmoid(float x) {
  if (x >= 0.0f) {
    const float z = std::exp(-x);
    return 1.0f / (1.0f + z);
  }
  const float z = std::exp(x);
  return z / (1.0f + z);
}

int argmax(const float *values, int count) {
  int best = 0;
  for (int i = 1; i < count; ++i) {
    if (values[i] > values[best]) best = i;
  }
  return best;
}

DecodeRect decode_rect_for_geometry(LdInputGeometry geometry) {
  if (geometry == LdInputGeometry::FullFrame) {
    return {};
  }
  return {kRoiX1, static_cast<float>(kLdRoiY1),
          kRoiX2, static_cast<float>(kLdRoiY2)};
}

bool has_meaningful_points(const std::vector<LanePoint> &points) {
  int valid_count = 0;
  bool has_nonzero_shape = false;
  for (const auto &point : points) {
    if (!lane_point_valid(point)) continue;
    ++valid_count;
    if (std::fabs(point.x) > 1e-4f || std::fabs(point.y) > 1e-4f) {
      has_nonzero_shape = true;
    }
  }
  return valid_count >= 2 && has_nonzero_shape;
}

std::vector<LanePoint> parse_points(int slot, const std::vector<float> &delta,
                                    const float *anchor, const float *point_conf,
                                    int frame_w, int frame_h, DecodeRect rect) {
  std::vector<int> valid_rows;
  for (int row = 0; row < kRowsPerLine; ++row) {
    const float confidence = point_conf[slot * kRowsPerLine + row];
    if (std::isfinite(confidence) && confidence >= 0.0f) valid_rows.push_back(row);
  }
  if (valid_rows.empty()) return {};

  const int crop_x1 = static_cast<int>(std::round(rect.x1 * frame_w));
  const int crop_y1 = static_cast<int>(std::round(rect.y1 * frame_h));
  const int crop_x2 = static_cast<int>(std::round(rect.x2 * frame_w));
  const int crop_y2 = static_cast<int>(std::round(rect.y2 * frame_h));
  const float crop_w = static_cast<float>(crop_x2 - crop_x1);
  const float crop_h = static_cast<float>(crop_y2 - crop_y1);

  auto nearest_idx = [&](float ratio) {
    const float target = 1.0f - ratio;
    int best = valid_rows[0];
    float best_dist = std::fabs(kPriorYs[best] - target);
    for (int row : valid_rows) {
      const float dist = std::fabs(kPriorYs[row] - target);
      if (dist < best_dist) {
        best = row;
        best_dist = dist;
      }
    }
    return best;
  };

  const float ext_start_ratio = anchor[slot * 4 + 0];
  const float ext_end_ratio = ext_start_ratio + anchor[slot * 4 + 1];
  const int start_idx = nearest_idx(ext_start_ratio);
  const int end_idx = nearest_idx(ext_end_ratio);
  const int lo = std::min(start_idx, end_idx);
  const int hi = std::max(start_idx, end_idx);

  std::vector<LanePoint> points;
  points.reserve(static_cast<size_t>(hi - lo + 1));
  for (int row = lo; row <= hi; ++row) {
    const float confidence = point_conf[slot * kRowsPerLine + row];
    if (!std::isfinite(confidence) || confidence < 0.0f) continue;

    const float x = delta[row] * crop_w + static_cast<float>(crop_x1);
    const float y = kPriorYs[row] * crop_h + static_cast<float>(crop_y1);
    float width_px = 0.0f;
    if (delta.size() >= kLaneDeltaSize && std::isfinite(delta[row + kLaneDeltaWidthOffset])) {
      width_px = delta[row + kLaneDeltaWidthOffset] * 2.0f * crop_w;
    }

    const float x_norm = x / static_cast<float>(frame_w);
    const float y_norm = y / static_cast<float>(frame_h);
    if (!std::isfinite(x_norm) || !std::isfinite(y_norm)) continue;

    points.push_back({
      x_norm,
      y_norm,
      confidence,
      std::isfinite(width_px) ? width_px : 0.0f,
      row,
      true,
    });
  }
  return points;
}

float mean_positive_conf(const float *point_conf, int slot) {
  float sum = 0.0f;
  int count = 0;
  for (int i = 0; i < kRowsPerLine; ++i) {
    const float v = point_conf[slot * kRowsPerLine + i];
    if (std::isfinite(v) && v > 0.0f) {
      sum += v;
      ++count;
    }
  }
  return count > 0 ? sum / static_cast<float>(count) : -1.0f;
}

}  // namespace

std::vector<DecodedLine> decode_ld_outputs(const std::vector<std::vector<float>> &outputs, int frame_w, int frame_h,
                                           LdInputGeometry geometry) {
  if (frame_w <= 0 || frame_h <= 0 ||
      outputs.size() != kOutputCount ||
      outputs[kLaneLogitsOutput].size() != kLaneSlots * kLaneLogitsPerSlot ||
      outputs[kBoundaryLogitsOutput].size() != kBoundarySlots * kBoundaryLogitsPerSlot ||
      outputs[kAnchorOutput].size() != (kLaneSlots + kBoundarySlots) * 4 ||
      outputs[kPointConfidenceOutput].size() != (kLaneSlots + kBoundarySlots) * kRowsPerLine) {
    return {};
  }

  const float *lane_logits = outputs[kLaneLogitsOutput].data();
  const float *boundary_logits = outputs[kBoundaryLogitsOutput].data();
  const float *anchor = outputs[kAnchorOutput].data();
  const float *point_conf = outputs[kPointConfidenceOutput].data();
  const DecodeRect decode_rect = decode_rect_for_geometry(geometry);
  std::vector<DecodedLine> decoded;
  decoded.reserve(10);

  for (int idx = 0; idx < kLaneSlots; ++idx) {
    const float lane_valid = sigmoid(lane_logits[idx * kLaneLogitsPerSlot + 0]);
    const float line_confidence = mean_positive_conf(point_conf, idx);
    if (line_confidence < 0.0f || lane_valid < 0.5f ||
        outputs[kLaneDeltaOutputStart + idx].size() != kLaneDeltaSize) {
      continue;
    }

    const int marker_pattern = argmax(lane_logits + idx * kLaneLogitsPerSlot + 1, 3);
    const int double_shape = argmax(lane_logits + idx * kLaneLogitsPerSlot + 4, 4);
    const int marker_color = argmax(lane_logits + idx * kLaneLogitsPerSlot + 8, 4);

    DecodedLine line;
    line.kind = DecodedLineKind::Lane;
    line.slot = idx;
    line.confidence = line_confidence;
    line.validity = lane_valid;
    line.marker_pattern = lane_marker_pattern_from_index(marker_pattern);
    line.double_shape = lane_double_shape_from_index(double_shape);
    line.raw_double_shape = double_shape;
    line.marker_color = lane_marker_color_from_index(marker_color);
    line.position_class = lane_position_from_slot(idx);
    line.points = parse_points(idx, outputs[kLaneDeltaOutputStart + idx], anchor, point_conf,
                               frame_w, frame_h, decode_rect);
    if (has_meaningful_points(line.points)) decoded.push_back(std::move(line));
  }

  for (int idx = 0; idx < kBoundarySlots; ++idx) {
    const int slot = kLaneSlots + idx;
    const float boundary_valid = sigmoid(boundary_logits[idx * kBoundaryLogitsPerSlot + 0]);
    const float line_confidence = mean_positive_conf(point_conf, slot);
    if (line_confidence < 0.0f || boundary_valid < 0.5f ||
        outputs[kBoundaryDeltaOutputStart + idx].size() != kBoundaryDeltaSize) {
      continue;
    }

    const int boundary_type = argmax(boundary_logits + idx * kBoundaryLogitsPerSlot + 1, 9);

    DecodedLine line;
    line.kind = DecodedLineKind::Boundary;
    line.slot = idx;
    line.confidence = line_confidence;
    line.validity = boundary_valid;
    line.boundary_position = idx;
    line.boundary_type = boundary_type_from_index(boundary_type);
    line.points = parse_points(slot, outputs[kBoundaryDeltaOutputStart + idx], anchor, point_conf,
                               frame_w, frame_h, decode_rect);
    if (has_meaningful_points(line.points)) decoded.push_back(std::move(line));
  }

  return decoded;
}

