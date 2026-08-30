#ifndef SUPERCOMBO_MODEL_H
#define SUPERCOMBO_MODEL_H

#include "ai_base.h"
#include "app_config.h"
#include "model_input_transform.h"
#include "model_output.h"

#include <cstddef>
#include <cstdint>
#include <vector>

class SupercomboModel : public AIBase
{
public:
    SupercomboModel(const char *kmodel_file, int debug_mode, const AppConfig &config);

    /* modeld가 프레임 링에서 이미 캐시 가능한 버퍼로 복사해 넘겨주므로
     * 여기서 다시 복사하지 않는다. C908 vluxei32.v가 /dev/shm 매핑 위에서
     * 불안정한 문제는 그 복사로 이미 해결된다. */
    bool run_frame_nv12(const uint8_t *nv12, int src_w, int src_h,
                        std::vector<float> &raw_output);
    void set_input_calibration(const float rpy[3]);
    void set_desire(int desire);

private:
    static constexpr int kModelW = 512;
    static constexpr int kModelH = 256;
    static constexpr int kHalfW = kModelW / 2;
    static constexpr int kHalfH = kModelH / 2;
    static constexpr int kYuv6Floats = 6 * kHalfW * kHalfH;
    static constexpr int kInputImageFloats = 12 * kHalfW * kHalfH;
    /* 시간축 규약(openpilot v0.9.4 driving.cc와 동일):
     * desire는 20 Hz 펄스 100틱 이력, 특징 버퍼는 직전 99틱 x 128. 둘 다
     * 최신 값이 마지막 슬롯이고, 매 프레임 한 칸씩 앞으로 밀린다. */
    static constexpr int kDesireHistoryTicks = 100;
    static constexpr int kFeatureHistoryTicks = 99;
    static constexpr int kNavFeatureLen = 256;

    bool prepare_image_input(size_t index, ModelInputTransform &transform,
                             const uint8_t *nv12, int src_w, int src_h);
    bool advance_image_history(size_t index);
    bool clear_image_input(size_t index);
    bool write_input(size_t index, const float *data, size_t count);
    size_t shape_count(size_t index) const;
    // image 입력 텐서의 원소 크기. float32 kmodel은 4, uint8 kmodel은 1이다.
    size_t image_elem_bytes(size_t index) const;

    void push_desire_pulse();
    void push_feature_history(const std::vector<float> &raw_output);
    bool write_temporal_inputs();

    std::vector<runtime_tensor> input_tensors_;
    ModelInputTransform input_transform_;
    ModelInputTransform big_input_transform_;
    std::vector<float> desire_;       // 현재 틱 펄스 (8)
    std::vector<float> prev_desire_;
    std::vector<float> traffic_convention_;
    std::vector<float> desire_history_;    // 펄스 이력 (100 x 8)
    std::vector<float> feature_history_;   // 특징 버퍼 (99 x 128)
    std::vector<float> nav_features_;      // 미사용 입력 (0 고정)
};

#endif
