#include "ld_model.h"

#include <nncase/runtime/runtime_op_utility.h>
#include <nncase/runtime/util.h>

#include <chrono>
#include <cstring>
#include <fstream>
#include <numeric>
#include <stdexcept>

using namespace nncase;
using namespace nncase::runtime;
using namespace nncase::runtime::k230;
using namespace nncase::F::k230;

namespace {

using Clock = std::chrono::steady_clock;

double elapsed_ms(Clock::time_point begin, Clock::time_point end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

size_t shape_count(const dims_t &shape)
{
    return std::accumulate(shape.begin(), shape.end(), size_t{1},
                           std::multiplies<size_t>());
}

// ROI 비율을 짝수 정렬된 소스 행 구간으로 바꾼다.
void ld_roi_rows(unsigned height, unsigned *row_begin, unsigned *row_end)
{
    unsigned begin = static_cast<unsigned>(height * kLdRoiY1) & ~1u;
    unsigned end = (static_cast<unsigned>(height * kLdRoiY2) + 1u) & ~1u;
    if (end > height) end = height;
    if (begin + 2u > end) begin = end > 2u ? end - 2u : 0u;
    *row_begin = begin;
    *row_end = end;
}

} // namespace

// kmodel을 로드하고 소스 크기에 맞는 변환기와 AI2D 스케줄을 준비한다.
LdModel::LdModel(const std::string &kmodel_path, unsigned src_width,
                 unsigned src_height)
    : src_width_(src_width), src_height_(src_height)
{
    std::ifstream model_stream(kmodel_path, std::ios::binary);
    if (!model_stream) throw std::runtime_error("cannot open LD kmodel: " + kmodel_path);
    interp_.load_model(model_stream).expect("invalid LD kmodel");

    if (interp_.inputs_size() != 1) throw std::runtime_error("LD kmodel must have 1 input");
    const auto input_desc = interp_.input_desc(0);
    const auto input_shape = interp_.input_shape(0);
    const dims_t expected{1, kLdInputHeight, kLdInputWidth, 3};
    if (input_shape != expected || input_desc.datatype != dt_uint8)
        throw std::runtime_error("LD kmodel input must be uint8 NHWC 1x320x640x3");

    input_tensor_ = host_runtime_tensor::create(input_desc.datatype, input_shape,
                                                hrt::pool_shared)
                        .expect("cannot create LD input tensor");
    interp_.input_tensor(0, input_tensor_).expect("cannot bind LD input tensor");
    for (size_t i = 0; i < interp_.outputs_size(); ++i) {
        auto desc = interp_.output_desc(i);
        auto tensor = host_runtime_tensor::create(desc.datatype, interp_.output_shape(i),
                                                  hrt::pool_shared)
                          .expect("cannot create LD output tensor");
        interp_.output_tensor(i, tensor).expect("cannot bind LD output tensor");
    }

    unsigned row_begin = 0;
    unsigned row_end = 0;
    ld_roi_rows(src_height_, &row_begin, &row_end);

    /* 기본: NV12 프레임을 AI2D에 직접 넣어 크롭+색변환+리사이즈를 하드웨어
     * 단일 패스로 처리한다(CPU 변환 ~4.7ms 제거, 매 프레임 LD의 전제).
     * SUPERCOMBO_LD_CPU_CONVERT=1이면 RVV CPU 변환 경로로 폴백(A/B 검증용). */
    if (const char *env = std::getenv("SUPERCOMBO_LD_CPU_CONVERT"))
        nv12_direct_ = !(env[0] == '1');

    dims_t ai2d_input_shape;
    ai2d_datatype_t dtype{ai2d_format::RGB_packed, ai2d_format::RGB_packed,
                          dt_uint8, dt_uint8};
    ai2d_crop_param_t crop_param{false, 0, 0, 0, 0};
    if (nv12_direct_) {
        /* AI2D의 NV12 입력은 높이 32배수 정렬을 요구한다(720 입력을 그대로
         * 주면 supercombo와 동시 구동 시 출력이 조용히 깨진다). ROI 하단
         * (0.85*h)이 32배수 절단선보다 위라 하단 행 절단은 손실이 없다. */
        ai2d_height_ = src_height_ & ~31u;
        if (row_end > ai2d_height_)
            throw std::runtime_error("LD ROI exceeds 32-aligned AI2D height");
        ai2d_input_shape = dims_t{1, 1, ai2d_height_ * 3 / 2, src_width_};
        dtype.src_format = ai2d_format::YUV420_NV12;
        crop_param = ai2d_crop_param_t{true, 0, static_cast<int>(row_begin),
                                       static_cast<int>(src_width_),
                                       static_cast<int>(row_end - row_begin)};
    } else {
        converter_.open(src_width_, src_height_, row_begin, row_end);
        ai2d_input_shape = dims_t{1, converter_.rgb_height(), src_width_, 3};
    }
    dims_t ai2d_output_shape = expected;
    ai2d_input_tensor_ = host_runtime_tensor::create(dt_uint8, ai2d_input_shape,
                                                     hrt::pool_shared)
                             .expect("cannot create LD AI2D input tensor");
    ai2d_shift_param_t shift_param{false, 0};
    ai2d_pad_param_t pad_param{false, {{0, 0}, {0, 0}, {0, 0}, {0, 0}},
                               ai2d_pad_mode::constant, {0, 0, 0}};
    ai2d_resize_param_t resize_param{true, ai2d_interp_method::tf_bilinear,
                                     ai2d_interp_mode::half_pixel};
    ai2d_affine_param_t affine_param{false, ai2d_interp_method::cv2_bilinear,
                                     0, 0, 127, 1,
                                     {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f}};
    ai2d_.reset(new ai2d_builder(ai2d_input_shape, ai2d_output_shape, dtype, crop_param,
                                 shift_param, pad_param, resize_param, affine_param));
    ai2d_->build_schedule().expect("LD AI2D build_schedule failed");
}

// NV12 프레임 하나를 추론해 디코드된 라인들을 반환한다.
std::vector<DecodedLine> LdModel::run(const uint8_t *nv12, LdModelTiming *timing)
{
    const auto t0 = Clock::now();
    if (!nv12_direct_) converter_.convert(nv12);
    const auto t1 = Clock::now();

    {
        auto host_buffer = ai2d_input_tensor_.impl()->to_host().unwrap()
                               ->buffer().as_host().unwrap();
        auto mapped = std::move(host_buffer.map(map_access_::map_write).unwrap());
        auto buffer = mapped.buffer();
        if (nv12_direct_) {
            /* Y 상단 ai2d_height_행 + 해당 UV행만 복사한다(높이 절단). */
            const size_t y_bytes = static_cast<size_t>(src_width_) * ai2d_height_;
            const size_t uv_bytes = y_bytes / 2;
            if (buffer.size() < y_bytes + uv_bytes)
                throw std::runtime_error("LD AI2D input buffer too small");
            std::memcpy(buffer.data(), nv12, y_bytes);
            std::memcpy(buffer.data() + y_bytes,
                        nv12 + static_cast<size_t>(src_width_) * src_height_, uv_bytes);
        } else {
            if (buffer.size() < converter_.rgb_bytes())
                throw std::runtime_error("LD AI2D input buffer too small");
            std::memcpy(buffer.data(), converter_.rgb(), converter_.rgb_bytes());
        }
    }
    hrt::sync(ai2d_input_tensor_, sync_op_t::sync_write_back, true)
        .expect("LD AI2D input sync failed");
    /* AI2D는 KPU(GNNE) 블록을 공유하므로 supercombo 실행과 직렬화해야 한다.
     * 뮤텍스 밖에서 invoke하면 동시 구동 시 출력이 조용히 깨진다. */
    Clock::time_point t2;
    if (npu_mutex_) {
        std::lock_guard<std::mutex> npu_lock(*npu_mutex_);
        ai2d_->invoke(ai2d_input_tensor_, input_tensor_).expect("LD AI2D invoke failed");
        t2 = Clock::now();
        interp_.run().expect("LD model run failed");
    } else {
        ai2d_->invoke(ai2d_input_tensor_, input_tensor_).expect("LD AI2D invoke failed");
        t2 = Clock::now();
        interp_.run().expect("LD model run failed");
    }
    std::vector<std::vector<float>> outputs = read_outputs();
    const auto t3 = Clock::now();

    /* 정규화 좌표를 원하므로 임의의 기준 크기로 디코드한다(parse_points가
     * frame 크기로 다시 나눠 0..1 좌표를 만든다). */
    std::vector<DecodedLine> lines =
        decode_ld_outputs(outputs, 1000, 1000, LdInputGeometry::RoiCrop);
    const auto t4 = Clock::now();

    if (timing) {
        timing->convert_ms = elapsed_ms(t0, t1);
        timing->ai2d_ms = elapsed_ms(t1, t2);
        timing->infer_ms = elapsed_ms(t2, t3);
        timing->decode_ms = elapsed_ms(t3, t4);
        timing->total_ms = elapsed_ms(t0, t4);
    }
    return lines;
}

// 모델 출력들을 float 벡터로 읽는다.
std::vector<std::vector<float>> LdModel::read_outputs()
{
    std::vector<std::vector<float>> outputs;
    outputs.reserve(interp_.outputs_size());
    for (size_t i = 0; i < interp_.outputs_size(); ++i) {
        auto out = interp_.output_tensor(i).expect("cannot get LD output tensor");
        hrt::sync(out, sync_op_t::sync_invalidate, true).expect("LD output sync failed");
        const size_t count = shape_count(interp_.output_shape(i));
        std::vector<float> values(count);
        auto host_buffer = out.impl()->to_host().unwrap()->buffer().as_host().unwrap();
        auto mapped = std::move(host_buffer.map(map_access_::map_read).unwrap());
        auto buffer = mapped.buffer();
        if (buffer.size() < count * sizeof(float))
            throw std::runtime_error("LD output buffer too small");
        std::memcpy(values.data(), buffer.data(), count * sizeof(float));
        outputs.push_back(std::move(values));
    }
    return outputs;
}
