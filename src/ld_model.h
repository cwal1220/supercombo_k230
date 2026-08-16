#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <nncase/functional/ai2d/ai2d_builder.h>
#include <nncase/runtime/interpreter.h>
#include <nncase/runtime/runtime_tensor.h>

#include "lane_decode.h"
#include "nv12_rgb_converter.h"

struct LdModelTiming {
    double convert_ms = 0.0;
    double ai2d_ms = 0.0;
    double infer_ms = 0.0;
    double decode_ms = 0.0;
    double total_ms = 0.0;
};

/* 차선 검출(LD) kmodel 러너. NV12 소스 프레임에서 ROI 행만 RGB로 변환하고
 * AI2D로 640x320에 리사이즈해 uint8 NHWC 입력으로 추론한 뒤 차선/경계
 * 라인을 디코드한다. supercombo 발행 직후(임계 경로 밖)에 저주기로 돈다. */
class LdModel {
public:
    // kmodel을 로드하고 소스 크기에 맞는 변환기와 AI2D 스케줄을 준비한다.
    LdModel(const std::string &kmodel_path, unsigned src_width, unsigned src_height);

    // NV12 프레임 하나를 추론해 디코드된 라인들을 반환한다.
    std::vector<DecodedLine> run(const uint8_t *nv12, LdModelTiming *timing = nullptr);

    // NPU 제출 직렬화용. supercombo와 공유한다.
    void set_npu_mutex(std::mutex *mutex) { npu_mutex_ = mutex; }

private:
    // 모델 출력들을 float 벡터로 읽는다.
    std::vector<std::vector<float>> read_outputs();

    nncase::runtime::interpreter interp_;
    bool nv12_direct_ = true;  // AI2D가 NV12->RGB 크롭/리사이즈까지 수행
    unsigned ai2d_height_ = 0;  // 32배수로 절단된 AI2D 입력 높이
    nncase::runtime::runtime_tensor input_tensor_;
    nncase::runtime::runtime_tensor ai2d_input_tensor_;
    std::unique_ptr<nncase::F::k230::ai2d_builder> ai2d_;
    Nv12RgbConverter converter_;
    unsigned src_width_ = 0;
    unsigned src_height_ = 0;
    std::mutex *npu_mutex_ = nullptr;
};
