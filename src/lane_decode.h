#pragma once

/* k230_adas의 차선 검출(LD) 모델 후처리 이식본. 640x320 RGB 입력 모델의
 * 15개 출력(차선 6슬롯 + 도로 경계 4슬롯 + 소실점)을 차선 종류/색상/이중선
 * 분류와 정규화 좌표 폴리라인으로 디코드한다. 그리기는 overlay 쪽 책임. */

#include <cmath>
#include <cstdint>
#include <vector>

/* LD 모델 입력 ROI: 전체 프레임 세로 [kLdRoiY1, kLdRoiY2] 구간을 잘라
 * 640x320으로 리사이즈한다. 비율은 센서 프레임 기준이라 소스 해상도와
 * 무관하게 같은 시야를 만든다. */
/* 밴드 높이 0.42963(=원본 0.37037~0.80)을 유지한 채 아래로 내려 하단 컷을
 * 15%로 만든다. 세로 압축비가 학습 파이프라인과 동일하게 유지된다. */
constexpr double kLdRoiY1 = 0.42037037;
constexpr double kLdRoiY2 = 0.85000000;
constexpr unsigned kLdInputWidth = 640;
constexpr unsigned kLdInputHeight = 320;

enum class DecodedLineKind {
  Lane,
  Boundary,
};

enum class LaneMarkerPattern {
  Solid = 0,
  Dashed = 1,
  BottsDots = 2,
  Unknown = 3,
};

enum class LaneMarkerColor {
  White = 0,
  Yellow = 1,
  Blue = 2,
  Unknown = 3,
};

enum class LaneDoubleShape {
  Class0 = 0,
  Class1 = 1,
  Class2 = 2,
  Class3 = 3,
  Unknown = 4,
};

enum class LanePosition {
  HostLaneLeft = 0,
  HostLaneRight = 1,
  LeftLaneRight = 2,
  RightLaneLeft = 3,
  LeftLaneLeft = 4,
  RightLaneRight = 5,
  Unknown = 6,
};

enum class BoundaryType {
  Unknown = 0,
  Edge = 1,
  Guardrail = 2,
  Barrier = 3,
  Curb = 4,
  Wall = 5,
  ParkedCars = 6,
  TrafficCones = 7,
  LaneSeparator = 8,
};

struct LanePoint {
  float x = 0.0f;
  float y = 0.0f;
  float confidence = 0.0f;
  float width_px = 0.0f;
  int row = 0;
  bool valid = true;
};

inline bool lane_point_valid(const LanePoint &point) {
  return point.valid &&
         std::isfinite(point.x) &&
         std::isfinite(point.y) &&
         std::isfinite(point.confidence) &&
         point.confidence >= 0.0f;
}

inline LaneMarkerPattern lane_marker_pattern_from_index(int index) {
  switch (index) {
    case 0: return LaneMarkerPattern::Solid;
    case 1: return LaneMarkerPattern::Dashed;
    case 2: return LaneMarkerPattern::BottsDots;
    default: return LaneMarkerPattern::Unknown;
  }
}

inline LaneMarkerColor lane_marker_color_from_index(int index) {
  switch (index) {
    case 0: return LaneMarkerColor::White;
    case 1: return LaneMarkerColor::Yellow;
    case 2: return LaneMarkerColor::Blue;
    default: return LaneMarkerColor::Unknown;
  }
}

inline LaneDoubleShape lane_double_shape_from_index(int index) {
  switch (index) {
    case 0: return LaneDoubleShape::Class0;
    case 1: return LaneDoubleShape::Class1;
    case 2: return LaneDoubleShape::Class2;
    case 3: return LaneDoubleShape::Class3;
    default: return LaneDoubleShape::Unknown;
  }
}

inline LanePosition lane_position_from_slot(int slot) {
  switch (slot) {
    case 0: return LanePosition::HostLaneLeft;
    case 1: return LanePosition::HostLaneRight;
    case 2: return LanePosition::LeftLaneRight;
    case 3: return LanePosition::RightLaneLeft;
    case 4: return LanePosition::LeftLaneLeft;
    case 5: return LanePosition::RightLaneRight;
    default: return LanePosition::Unknown;
  }
}

inline BoundaryType boundary_type_from_index(int index) {
  switch (index) {
    case 0: return BoundaryType::Unknown;
    case 1: return BoundaryType::Edge;
    case 2: return BoundaryType::Guardrail;
    case 3: return BoundaryType::Barrier;
    case 4: return BoundaryType::Curb;
    case 5: return BoundaryType::Wall;
    case 6: return BoundaryType::ParkedCars;
    case 7: return BoundaryType::TrafficCones;
    case 8: return BoundaryType::LaneSeparator;
    default: return BoundaryType::Unknown;
  }
}

inline int lane_class_index(LaneMarkerPattern value) {
  return static_cast<int>(value);
}

inline int lane_class_index(LaneMarkerColor value) {
  return static_cast<int>(value);
}

inline int lane_class_index(LaneDoubleShape value) {
  return static_cast<int>(value);
}

inline int lane_class_index(BoundaryType value) {
  return static_cast<int>(value);
}

struct DecodedLine {
  DecodedLineKind kind = DecodedLineKind::Lane;
  int slot = 0;
  std::vector<LanePoint> points;
  float confidence = 0.0f;
  float validity = 0.0f;
  LaneDoubleShape double_shape = LaneDoubleShape::Unknown;
  int raw_double_shape = -1;
  LaneMarkerPattern marker_pattern = LaneMarkerPattern::Unknown;
  LaneMarkerColor marker_color = LaneMarkerColor::Unknown;
  LanePosition position_class = LanePosition::Unknown;
  int boundary_position = 0;
  BoundaryType boundary_type = BoundaryType::Unknown;
};

enum class LdInputGeometry {
  FullFrame,
  RoiCrop,
};

std::vector<DecodedLine> decode_ld_outputs(const std::vector<std::vector<float>> &outputs, int frame_w, int frame_h,
                                           LdInputGeometry geometry = LdInputGeometry::RoiCrop);
