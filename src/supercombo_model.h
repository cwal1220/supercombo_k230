#ifndef SUPERCOMBO_MODEL_H
#define SUPERCOMBO_MODEL_H

#include "app_config.h"
#include "gpu_warp.h"
#include "model_input_transform.h"
#include "model_output.h"
#include "model_temporal.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <nncase/runtime/interpreter.h>
#include <nncase/runtime/runtime_op_utility.h>
#include <nncase/runtime/util.h>

/* kmodel 로드·실행을 직접 감싼다. nncase 심볼은 이 클래스 안에만 둔다. */
class SupercomboModel
{
public:
    SupercomboModel(const char *kmodel_file, int debug_mode, const AppConfig &config);

    /* modeld가 프레임 링에서 이미 캐시 가능한 버퍼로 복사해 넘겨주므로
     * 여기서 다시 복사하지 않는다. C908 vluxei32.v가 /dev/shm 매핑 위에서
     * 불안정한 문제는 그 복사로 이미 해결된다. */
    bool run_frame_nv12(const uint8_t *nv12, int src_w, int src_h,
                        std::vector<float> &raw_output);
    /* GPU 워프를 쓰는 경우 소비자가 프레임을 소스 평면에 직접 채울 수 있다.
     * 그렇게 채웠으면 run_frame_preloaded로 실행한다. */
    bool frame_planes(GpuWarp::Planes *planes);
    bool run_frame_preloaded(int src_w, int src_h, std::vector<float> &raw_output);
    void set_input_calibration(const float rpy[3]);
    void set_desire(int desire);

private:
    bool run_frame(const uint8_t *nv12, int src_w, int src_h, std::vector<float> &raw_output);
    void setup_gpu(const AppConfig &config);
    bool prepare_images_gpu(const uint8_t *nv12);
    void bind_input_tensors();
    void bind_output_tensors();
    void run();
    // 출력 텐서들을 순서대로 이어 raw_output에 복사한다. 매핑은 복사 동안만 산다.
    bool copy_outputs(std::vector<float> &raw_output);

    static constexpr int kModelW = 512;
    static constexpr int kModelH = 256;
    static constexpr int kHalfW = kModelW / 2;
    static constexpr int kHalfH = kModelH / 2;
    static constexpr int kYuv6Floats = 6 * kHalfW * kHalfH;
    static constexpr int kInputImageFloats = 12 * kHalfW * kHalfH;

    /* 입력 텐서 index를 access로 매핑해 fn(uint8_t *data)를 부르고 언매핑한다.
     * 버퍼가 min_bytes보다 작으면 fn을 부르지 않고 false. */
    template <class Fn>
    bool with_mapped_input(size_t index, nncase::runtime::map_access_t access,
                           size_t min_bytes, Fn &&fn);
    bool prepare_image_input(size_t index, ModelInputTransform &transform,
                             const uint8_t *nv12, int src_w, int src_h);
    bool advance_image_history(size_t index);
    bool clear_image_input(size_t index);
    bool write_input(size_t index, const float *data, size_t count);
    size_t shape_count(size_t index) const;
    // image 입력 텐서의 원소 크기. float32 kmodel은 4, uint8 kmodel은 1이다.
    size_t image_elem_bytes(size_t index) const;

    // traffic convention과 nav features는 상수라 생성 시 한 번만 쓴다.
    bool write_constant_inputs();
    bool write_temporal_inputs();

    nncase::runtime::interpreter kmodel_interp_;
    int debug_mode_ = 0;
    std::vector<std::vector<int>> input_shapes_;
    std::vector<std::vector<int>> output_shapes_;
    std::vector<nncase::runtime::runtime_tensor> input_tensors_;
    ModelInputTransform input_transform_;
    ModelInputTransform big_input_transform_;
    SupercomboTemporalState temporal_;
    std::unique_ptr<GpuWarp> gpu_;
    /* GPU 타깃은 텐서 주소에 고정이라 매핑을 유지한다. */
    std::vector<nncase::runtime::mapped_buffer> image_maps_;
    bool gpu_projection_dirty_ = true;
};

#endif
