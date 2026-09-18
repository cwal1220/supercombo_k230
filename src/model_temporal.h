#ifndef MODEL_TEMPORAL_H
#define MODEL_TEMPORAL_H

/* supercombo의 시간축 입력(openpilot v0.9.4 driving.cc와 동일): desire는 20 Hz
 * 펄스 100틱 이력, 특징 버퍼는 직전 99틱 x 128. 둘 다 최신 값이 마지막 슬롯이고
 * 매 프레임 한 칸씩 앞으로 밀린다. traffic convention과 nav features는 이 차에서
 * 상수다. nncase에 의존하지 않아 호스트 검사가 규약을 본다. */

#include "model_output.h"

#include <cstddef>
#include <cstring>
#include <vector>

class SupercomboTemporalState
{
public:
    static constexpr int kDesireHistoryTicks = 100;
    static constexpr int kFeatureHistoryTicks = 99;
    static constexpr int kNavFeatureLen = 256;
    static constexpr size_t kHiddenOffset = model_output_layout::kFeatureOffset;

    SupercomboTemporalState()
        : desire_(kDesireLen, 0.0f),
          prev_desire_(kDesireLen, 0.0f),
          traffic_convention_{1.0f, 0.0f},
          desire_history_(kDesireHistoryTicks * kDesireLen, 0.0f),
          feature_history_(kFeatureHistoryTicks * kModelFeatureLen, 0.0f),
          nav_features_(kNavFeatureLen, 0.0f)
    {
    }

    /* openpilot 방식 rising-edge 펄스: desire가 바뀐 틱에만 1. 0(없음)은 항상 0. */
    void set_desire(int desire)
    {
        for (int i = 1; i < static_cast<int>(desire_.size()); ++i) {
            const float current = i == desire ? 1.0f : 0.0f;
            desire_[i] = current - prev_desire_[i] > 0.99f ? current : 0.0f;
            prev_desire_[i] = current;
        }
    }

    // 오래된 틱을 앞으로 밀고 마지막 슬롯에 현재 펄스를 넣는다. 매 프레임 실행 직전.
    void push_desire_pulse()
    {
        std::memmove(desire_history_.data(), desire_history_.data() + kDesireLen,
                     sizeof(float) * kDesireLen * (kDesireHistoryTicks - 1));
        std::memcpy(desire_history_.data() + kDesireLen * (kDesireHistoryTicks - 1),
                    desire_.data(), sizeof(float) * kDesireLen);
    }

    // 실행 직후 raw 출력의 hidden_state를 특징 버퍼 마지막 슬롯에 넣는다.
    bool push_feature_history(const float *raw_output, size_t count)
    {
        if (count < kHiddenOffset + kModelFeatureLen) return false;
        std::memmove(feature_history_.data(), feature_history_.data() + kModelFeatureLen,
                     sizeof(float) * kModelFeatureLen * (kFeatureHistoryTicks - 1));
        std::memcpy(feature_history_.data() + kModelFeatureLen * (kFeatureHistoryTicks - 1),
                    raw_output + kHiddenOffset, sizeof(float) * kModelFeatureLen);
        return true;
    }

    const std::vector<float> &desire_history() const { return desire_history_; }
    const std::vector<float> &feature_history() const { return feature_history_; }
    const std::vector<float> &traffic_convention() const { return traffic_convention_; }
    const std::vector<float> &nav_features() const { return nav_features_; }

private:
    std::vector<float> desire_;
    std::vector<float> prev_desire_;
    std::vector<float> traffic_convention_;
    std::vector<float> desire_history_;
    std::vector<float> feature_history_;
    std::vector<float> nav_features_;
};

#endif
