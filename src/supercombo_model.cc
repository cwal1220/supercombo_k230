#include "supercombo_model.h"

#include "common_utils.h"
#include "scoped_timing.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <numeric>
#include <stdexcept>

using namespace nncase::runtime;

namespace {

uint64_t now_ns()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool profile_enabled()
{
    static const bool enabled = [] {
        const char *value = std::getenv("SUPERCOMBO_PROFILE");
        return value && value[0] != '\0' && std::strcmp(value, "0") != 0;
    }();
    return enabled;
}

double ns_to_ms(uint64_t ns)
{
    return static_cast<double>(ns) / 1000000.0;
}

struct ProfileStats {
    uint64_t count = 0;
    double preprocess_ms = 0.0;
    double input_ms = 0.0;
    double run_ms = 0.0;
    double history_ms = 0.0;
    double output_ms = 0.0;
    double total_ms = 0.0;

    void add(uint64_t preprocess_ns, uint64_t input_ns, uint64_t run_ns,
             uint64_t history_ns, uint64_t output_ns, uint64_t total_ns)
    {
        ++count;
        preprocess_ms += ns_to_ms(preprocess_ns);
        input_ms += ns_to_ms(input_ns);
        run_ms += ns_to_ms(run_ns);
        history_ms += ns_to_ms(history_ns);
        output_ms += ns_to_ms(output_ns);
        total_ms += ns_to_ms(total_ns);

        if (count % 30 == 0) {
            const double denom = static_cast<double>(count);
            std::fprintf(stderr,
                         "\nprofile avg[%llu] ms: preprocess=%.3f input=%.3f run=%.3f history=%.3f output=%.3f total=%.3f\n",
                         static_cast<unsigned long long>(count),
                         preprocess_ms / denom,
                         input_ms / denom,
                         run_ms / denom,
                         history_ms / denom,
                         output_ms / denom,
                         total_ms / denom);
            std::fflush(stderr);
        }
    }
};

ProfileStats &profile_stats()
{
    static ProfileStats stats;
    return stats;
}

} // namespace

// nncase 입력 텐서를 만들어 바인딩하고 shape를 기억한다.
void SupercomboModel::bind_input_tensors()
{
    ScopedTiming st("Supercombo set_input init", debug_mode_);
    for (size_t i = 0; i < kmodel_interp_.inputs_size(); ++i) {
        auto desc = kmodel_interp_.input_desc(i);
        auto shape = kmodel_interp_.input_shape(i);
        auto tensor = nncase::runtime::host_runtime_tensor::create(
            desc.datatype, shape, nncase::runtime::hrt::pool_shared)
                          .expect("cannot create input tensor");
        kmodel_interp_.input_tensor(i, tensor).expect("cannot set input tensor");
        std::vector<int> dims;
        for (size_t j = 0; j < shape.size(); ++j)
            dims.push_back(static_cast<int>(shape[j]));
        input_shapes_.push_back(dims);
        input_tensors_.push_back(
            kmodel_interp_.input_tensor(i).expect("cannot get input tensor"));
    }
}

void SupercomboModel::bind_output_tensors()
{
    ScopedTiming st("Supercombo set_output init", debug_mode_);
    for (size_t i = 0; i < kmodel_interp_.outputs_size(); ++i) {
        auto desc = kmodel_interp_.output_desc(i);
        auto shape = kmodel_interp_.output_shape(i);
        std::vector<int> dims;
        for (size_t j = 0; j < shape.size(); ++j)
            dims.push_back(static_cast<int>(shape[j]));
        output_shapes_.push_back(dims);
        auto tensor = nncase::runtime::host_runtime_tensor::create(
            desc.datatype, shape, nncase::runtime::hrt::pool_shared)
                          .expect("cannot create output tensor");
        kmodel_interp_.output_tensor(i, tensor).expect("cannot set output tensor");
    }
}

void SupercomboModel::run()
{
    ScopedTiming st("Supercombo run", debug_mode_);
    kmodel_interp_.run().expect("error occurred in running model");
}

void SupercomboModel::fetch_outputs()
{
    ScopedTiming st("Supercombo get_output", debug_mode_);
    outputs_.clear();
    for (size_t i = 0; i < kmodel_interp_.outputs_size(); ++i) {
        auto out = kmodel_interp_.output_tensor(i).expect("cannot get output tensor");
        auto buf = out.impl()->to_host().unwrap()->buffer().as_host().unwrap()
                       .map(nncase::runtime::map_access_::map_read).unwrap().buffer();
        outputs_.push_back(reinterpret_cast<float *>(buf.data()));
    }
}

SupercomboModel::SupercomboModel(const char *kmodel_file, int debug_mode, const AppConfig &config)
    : debug_mode_(debug_mode),
      input_transform_(config, ModelFrame::MedModel),
      big_input_transform_(config, ModelFrame::SmallBigModel),
      desire_(kDesireLen, 0.0f),
      prev_desire_(kDesireLen, 0.0f),
      traffic_convention_{1.0f, 0.0f},
      desire_history_(kDesireHistoryTicks * kDesireLen, 0.0f),
      feature_history_(kFeatureHistoryTicks * kModelFeatureLen, 0.0f),
      nav_features_(kNavFeatureLen, 0.0f)
{
    std::ifstream kmodel(kmodel_file, std::ios::binary);
    kmodel_interp_.load_model(kmodel).expect("Invalid kmodel");
    bind_input_tensors();
    bind_output_tensors();
    for (size_t i = 0; i < 2; ++i) {
        const auto image_type = input_tensors_[i].datatype();
        if (image_type != nncase::dt_float32 && image_type != nncase::dt_uint8)
            throw std::runtime_error("unsupported image input datatype");
    }
    if (input_tensors_[0].datatype() != input_tensors_[1].datatype())
        throw std::runtime_error("image input datatype mismatch");

    /* openpilot v0.9.4 계약을 강제한다: 이미지 2 + desire 이력 + traffic
     * convention + nav + 특징 버퍼 = 6입력, 출력 6120. 다른 kmodel은 조용히
     * 오해석되지 않도록 여기서 거부한다. */
    size_t output_floats = 0;
    for (const auto &shape : output_shapes_)
        output_floats += std::accumulate(shape.begin(), shape.end(), size_t{1},
                                         std::multiplies<size_t>());
    if (input_tensors_.size() != 6 || output_floats != kModelOutputFloats ||
        shape_count(2) != static_cast<size_t>(kDesireHistoryTicks * kDesireLen) ||
        shape_count(4) != static_cast<size_t>(kNavFeatureLen) ||
        shape_count(5) != static_cast<size_t>(kFeatureHistoryTicks * kModelFeatureLen)) {
        throw std::runtime_error("not an openpilot v0.9.4 supercombo kmodel: " +
                                 std::to_string(input_tensors_.size()) +
                                 " inputs, " + std::to_string(output_floats) +
                                 " output floats");
    }

    std::fprintf(stderr, "Supercombo openpilot-v0.9.4 image dtype=%s\n",
                 input_tensors_[0].datatype() == nncase::dt_uint8 ? "uint8" : "float32");
    if (!clear_image_input(0) || !clear_image_input(1))
        throw std::runtime_error("initialize image input history failed");

    setup_gpu(config);
}

/* 워프를 VGLite로 넘긴다. 출력 6평면을 모델 입력 텐서의 뒤쪽 절반(현재 프레임)
 * 물리주소에 직접 묶으므로 복사가 없다. 실패하면 CPU 경로를 그대로 쓴다. */
void SupercomboModel::setup_gpu(const AppConfig &config)
{
    if (env_flag("SUPERCOMBO_WARP_CPU", false))
        return;

    gpu_ = GpuWarp::create(static_cast<int>(config.nv12_width),
                           static_cast<int>(config.nv12_height));
    if (!gpu_)
        return;

    for (size_t i = 0; i < 2; ++i) {
        auto host_buffer = input_tensors_[i].impl()->to_host().unwrap()
                               ->buffer().as_host().unwrap();
        if (!host_buffer.has_physical_address()) {
            std::fprintf(stderr, "gpu warp: image tensor has no physical address\n");
            gpu_.reset();
            image_maps_.clear();
            return;
        }
        const uintptr_t physical = host_buffer.physical_address().expect("image physical address");
        auto mapped = host_buffer.map(map_access_::map_read_write).unwrap();
        uint8_t *base = reinterpret_cast<uint8_t *>(mapped.buffer().data());
        image_maps_.push_back(std::move(mapped));
        if (!gpu_->bind_output(static_cast<int>(i), base + kYuv6Floats, physical + kYuv6Floats)) {
            gpu_.reset();
            image_maps_.clear();
            return;
        }
    }

    if (!gpu_)
        return;
    std::fprintf(stderr, "Supercombo warp backend=vglite\n");
}

void SupercomboModel::push_desire_pulse()
{
    // 오래된 틱을 앞으로 밀고 마지막 슬롯에 현재 펄스를 넣는다.
    std::memmove(desire_history_.data(), desire_history_.data() + kDesireLen,
                 sizeof(float) * kDesireLen * (kDesireHistoryTicks - 1));
    std::memcpy(desire_history_.data() + kDesireLen * (kDesireHistoryTicks - 1),
                desire_.data(), sizeof(float) * kDesireLen);
}

void SupercomboModel::push_feature_history(const std::vector<float> &raw_output)
{
    // hidden_state는 마지막 2 float(pad) 앞의 128개다.
    constexpr size_t kHiddenOffset = kModelOutputFloats - 2u - kModelFeatureLen;
    if (raw_output.size() < kHiddenOffset + kModelFeatureLen) return;
    std::memmove(feature_history_.data(), feature_history_.data() + kModelFeatureLen,
                 sizeof(float) * kModelFeatureLen * (kFeatureHistoryTicks - 1));
    std::memcpy(feature_history_.data() + kModelFeatureLen * (kFeatureHistoryTicks - 1),
                raw_output.data() + kHiddenOffset, sizeof(float) * kModelFeatureLen);
}

bool SupercomboModel::write_temporal_inputs()
{
    push_desire_pulse();
    if (!write_input(2, desire_history_.data(), desire_history_.size())) return false;
    if (!write_input(3, traffic_convention_.data(), traffic_convention_.size())) return false;
    if (!write_input(4, nav_features_.data(), nav_features_.size())) return false;
    return write_input(5, feature_history_.data(), feature_history_.size());
}

size_t SupercomboModel::image_elem_bytes(size_t index) const
{
    return input_tensors_[index].datatype() == nncase::dt_uint8 ? 1 : sizeof(float);
}

void SupercomboModel::set_desire(int desire)
{
    for (int i = 1; i < static_cast<int>(desire_.size()); ++i) {
        const float current = i == desire ? 1.0f : 0.0f;
        desire_[i] = current - prev_desire_[i] > 0.99f ? current : 0.0f;
        prev_desire_[i] = current;
    }
}

void SupercomboModel::set_input_calibration(const float rpy[3])
{
    input_transform_.set_calibration(rpy[0], rpy[1], rpy[2]);
    big_input_transform_.set_calibration(rpy[0], rpy[1], rpy[2]);
    gpu_projection_dirty_ = true;
}

bool SupercomboModel::frame_planes(GpuWarp::Planes *planes)
{
    if (!gpu_ || !planes)
        return false;
    *planes = gpu_->planes();
    return planes->luma != nullptr;
}

/* 히스토리는 CPU가 쓴 뒤라 먼저 DDR로 밀어내고, GPU가 쓴 뒤에는 무효화해야
 * stale 캐시 라인이 워프 결과를 덮지 않는다. */
bool SupercomboModel::prepare_images_gpu(const uint8_t *nv12)
{
    if (gpu_projection_dirty_) {
        float projection[9];
        input_transform_.projection_matrix(projection);
        gpu_->set_projection(0, projection);
        big_input_transform_.projection_matrix(projection);
        gpu_->set_projection(1, projection);
        gpu_projection_dirty_ = false;
    }

    for (size_t i = 0; i < 2; ++i)
        hrt::sync(input_tensors_[i], sync_op_t::sync_write_back, true)
            .expect("sync image write back failed");
    if (nv12)
        gpu_->upload(nv12);
    else
        gpu_->split_chroma();
    if (!gpu_->run())
        return false;
    for (size_t i = 0; i < 2; ++i)
        hrt::sync(input_tensors_[i], sync_op_t::sync_invalidate, true)
            .expect("sync image invalidate failed");
    return true;
}

bool SupercomboModel::run_frame_nv12(const uint8_t *nv12, int src_w, int src_h,
                                     std::vector<float> &raw_output)
{
    if (!nv12)
        return false;
    return run_frame(nv12, src_w, src_h, raw_output);
}

bool SupercomboModel::run_frame_preloaded(int src_w, int src_h, std::vector<float> &raw_output)
{
    if (!gpu_)
        return false;
    return run_frame(nullptr, src_w, src_h, raw_output);
}

bool SupercomboModel::run_frame(const uint8_t *nv12, int src_w, int src_h,
                                std::vector<float> &raw_output)
{
    if (src_w <= 0 || src_h <= 0)
        return false;

    const bool gpu = gpu_ && gpu_->accepts(src_w, src_h);
    if (!gpu && !nv12)
        return false;

    const bool profile = profile_enabled();
    const uint64_t t1 = profile ? now_ns() : 0;
    if (gpu) {
        if (!prepare_images_gpu(nv12)) return false;
    } else {
        if (!prepare_image_input(0, input_transform_, nv12, src_w, src_h)) return false;
        if (!prepare_image_input(1, big_input_transform_, nv12, src_w, src_h)) return false;
        hrt::sync(input_tensors_[0], sync_op_t::sync_write_back, true).expect("sync input 0 failed");
        hrt::sync(input_tensors_[1], sync_op_t::sync_write_back, true).expect("sync input 1 failed");
    }
    const uint64_t t2 = profile ? now_ns() : 0;

    if (!write_temporal_inputs()) return false;
    const uint64_t t3 = profile ? now_ns() : 0;

    run();
    const uint64_t t4 = profile ? now_ns() : 0;
    if (!advance_image_history(0) || !advance_image_history(1)) return false;
    const uint64_t t5 = profile ? now_ns() : 0;
    fetch_outputs();

    size_t total = 0;
    for (const auto &shape : output_shapes_)
        total += std::accumulate(shape.begin(), shape.end(), size_t{1}, std::multiplies<size_t>());

    raw_output.resize(total);
    size_t offset = 0;
    for (size_t i = 0; i < output_shapes_.size(); ++i) {
        const size_t count = std::accumulate(output_shapes_[i].begin(), output_shapes_[i].end(), size_t{1}, std::multiplies<size_t>());
        std::memcpy(raw_output.data() + offset, outputs_[i], count * sizeof(float));
        offset += count;
    }

    push_feature_history(raw_output);

    if (profile) {
        const uint64_t t6 = now_ns();
        profile_stats().add(t2 - t1, t3 - t2, t4 - t3, t5 - t4, t6 - t5, t6 - t1);
    }

    return true;
}

bool SupercomboModel::prepare_image_input(size_t index, ModelInputTransform &transform,
                                          const uint8_t *nv12, int src_w, int src_h)
{
    if (index >= input_tensors_.size() || shape_count(index) != kInputImageFloats)
        return false;

    const size_t elem_bytes = image_elem_bytes(index);
    auto host_buffer = input_tensors_[index].impl()->to_host().unwrap()
                           ->buffer().as_host().unwrap();
    auto mapped = std::move(host_buffer.map(map_access_::map_read_write).unwrap());
    auto buffer = mapped.buffer();
    if (buffer.size() < kInputImageFloats * elem_bytes)
        return false;

    if (elem_bytes == 1) {
        uint8_t *input = reinterpret_cast<uint8_t *>(buffer.data());
        transform.nv12_to_yuv6_warped(nv12, src_w, src_h, input + kYuv6Floats);
    } else {
        float *input = reinterpret_cast<float *>(buffer.data());
        transform.nv12_to_yuv6_warped(nv12, src_w, src_h, input + kYuv6Floats);
    }
    mapped.unmap().expect("unmap image input failed");
    return true;
}

bool SupercomboModel::advance_image_history(size_t index)
{
    if (index >= input_tensors_.size() || shape_count(index) != kInputImageFloats)
        return false;

    const size_t elem_bytes = image_elem_bytes(index);
    auto host_buffer = input_tensors_[index].impl()->to_host().unwrap()
                           ->buffer().as_host().unwrap();
    auto mapped = std::move(host_buffer.map(map_access_::map_read_write).unwrap());
    auto buffer = mapped.buffer();
    if (buffer.size() < kInputImageFloats * elem_bytes)
        return false;

    uint8_t *input = reinterpret_cast<uint8_t *>(buffer.data());
    std::memcpy(input, input + kYuv6Floats * elem_bytes, kYuv6Floats * elem_bytes);
    mapped.unmap().expect("unmap image history failed");
    return true;
}

bool SupercomboModel::clear_image_input(size_t index)
{
    if (index >= input_tensors_.size() || shape_count(index) != kInputImageFloats)
        return false;

    const size_t elem_bytes = image_elem_bytes(index);
    auto host_buffer = input_tensors_[index].impl()->to_host().unwrap()
                           ->buffer().as_host().unwrap();
    auto mapped = std::move(host_buffer.map(map_access_::map_write).unwrap());
    auto buffer = mapped.buffer();
    if (buffer.size() < kInputImageFloats * elem_bytes)
        return false;

    std::memset(buffer.data(), 0, kInputImageFloats * elem_bytes);
    mapped.unmap().expect("unmap cleared image input failed");
    hrt::sync(input_tensors_[index], sync_op_t::sync_write_back, true)
        .expect("sync cleared image input failed");
    return true;
}

bool SupercomboModel::write_input(size_t index, const float *data, size_t count)
{
    if (index >= input_tensors_.size()) {
        std::cerr << "missing input tensor " << index << std::endl;
        return false;
    }
    const size_t expected = shape_count(index);
    if (expected != count) {
        std::cerr << "input " << index << " shape count mismatch: model=" << expected
                  << " app=" << count << std::endl;
        return false;
    }

    auto buf = input_tensors_[index].impl()->to_host().unwrap()->buffer().as_host().unwrap()
                   .map(map_access_::map_write).unwrap().buffer();
    if (buf.size() < count * sizeof(float)) {
        std::cerr << "input " << index << " buffer too small: " << buf.size()
                  << " < " << count * sizeof(float) << std::endl;
        return false;
    }
    std::memcpy(reinterpret_cast<char *>(buf.data()), data, count * sizeof(float));
    hrt::sync(input_tensors_[index], sync_op_t::sync_write_back, true).expect("sync write_back failed");
    return true;
}

size_t SupercomboModel::shape_count(size_t index) const
{
    if (index >= input_shapes_.size()) return 0;
    return std::accumulate(input_shapes_[index].begin(), input_shapes_[index].end(),
                           size_t{1}, std::multiplies<size_t>());
}
