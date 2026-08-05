#include "supercombo_model.h"

#include "lane_plan_fusion.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <vector>

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
    double source_copy_ms = 0.0;
    double preprocess_ms = 0.0;
    double input_ms = 0.0;
    double run_ms = 0.0;
    double history_ms = 0.0;
    double output_ms = 0.0;
    double total_ms = 0.0;
    std::vector<double> total_samples_ms;

    static double percentile(std::vector<double> values, double fraction)
    {
        if (values.empty()) return 0.0;
        std::sort(values.begin(), values.end());
        const size_t index = static_cast<size_t>(
            std::ceil(fraction * static_cast<double>(values.size())) - 1.0);
        return values[std::min(index, values.size() - 1)];
    }

    void add(uint64_t source_copy_ns, uint64_t preprocess_ns, uint64_t input_ns,
             uint64_t run_ns, uint64_t history_ns, uint64_t output_ns,
             uint64_t total_ns)
    {
        ++count;
        source_copy_ms += ns_to_ms(source_copy_ns);
        preprocess_ms += ns_to_ms(preprocess_ns);
        input_ms += ns_to_ms(input_ns);
        run_ms += ns_to_ms(run_ns);
        history_ms += ns_to_ms(history_ns);
        output_ms += ns_to_ms(output_ns);
        total_ms += ns_to_ms(total_ns);
        total_samples_ms.push_back(ns_to_ms(total_ns));

        if (count % 30 == 0) {
            const double denom = static_cast<double>(count);
            const size_t warmup = std::min<size_t>(5, total_samples_ms.size());
            std::vector<double> steady(total_samples_ms.begin() + warmup,
                                       total_samples_ms.end());
            const size_t deadline_misses = static_cast<size_t>(std::count_if(
                steady.begin(), steady.end(), [](double value) { return value > 50.0; }));
            std::fprintf(stderr,
                         "\nprofile avg[%llu] ms: source_copy=%.3f preprocess=%.3f input=%.3f run=%.3f history=%.3f output=%.3f total=%.3f p95=%.3f p99=%.3f max=%.3f over50=%zu/%zu\n",
                         static_cast<unsigned long long>(count), source_copy_ms / denom,
                         preprocess_ms / denom, input_ms / denom, run_ms / denom,
                         history_ms / denom, output_ms / denom, total_ms / denom,
                         percentile(steady, 0.95), percentile(steady, 0.99),
                         steady.empty() ? 0.0 : *std::max_element(steady.begin(), steady.end()),
                         deadline_misses, steady.size());
            std::fflush(stderr);
        }
    }
};

ProfileStats &profile_stats()
{
    static ProfileStats stats;
    return stats;
}

bool shape_is(const std::vector<int> &shape, std::initializer_list<int> expected)
{
    return shape.size() == expected.size() &&
        std::equal(shape.begin(), shape.end(), expected.begin());
}

} // namespace

SupercomboModel::SupercomboModel(const char *kmodel_file, int debug_mode, const AppConfig &config)
    : AIBase(kmodel_file, "Supercombo", debug_mode),
      input_transform_(config, ModelFrame::MedModel),
      big_input_transform_(config, ModelFrame::SmallBigModel),
      lane_plan_fusion_enabled_(config.lane_plan_fusion)
{
    if (!std::isfinite(config.lateral_action_t) ||
        !std::isfinite(config.longitudinal_action_t) ||
        config.lateral_action_t < 0.0f || config.lateral_action_t > 2.0f ||
        config.longitudinal_action_t < 0.0f || config.longitudinal_action_t > 2.0f) {
        throw std::invalid_argument("action_t must be finite and between 0 and 2 seconds");
    }

    validate_model_abi();
    for (size_t i = 0; i < input_shapes_.size(); ++i)
        input_tensors_.push_back(get_input_tensor(i));

    const float traffic[2] = {1.0f, 0.0f};
    const float action[2] = {config.lateral_action_t, config.longitudinal_action_t};
    model_encode_float16(traffic, 2, traffic_input_.data());
    model_encode_float16(action, 2, action_input_.data());
    std::fprintf(stderr, "lane-plan fusion=%s\n",
                 lane_plan_fusion_enabled_ ? "enabled" : "disabled");
    reset_temporal_state();
}

void SupercomboModel::validate_model_abi() const
{
    if (input_shapes_.size() != 6 || input_dtypes_.size() != 6 ||
        output_shapes_.size() != 1 || output_dtypes_.size() != 1) {
        throw std::runtime_error("modern supercombo ABI requires exactly 6 inputs and 1 output");
    }

    const bool inputs_ok =
        shape_is(input_shapes_[0], {1, 12, 128, 256}) && input_dtypes_[0] == nncase::dt_uint8 &&
        shape_is(input_shapes_[1], {1, 12, 128, 256}) && input_dtypes_[1] == nncase::dt_uint8 &&
        shape_is(input_shapes_[2], {1, 24, 512}) && input_dtypes_[2] == nncase::dt_float16 &&
        shape_is(input_shapes_[3], {1, 25, 8}) && input_dtypes_[3] == nncase::dt_float16 &&
        shape_is(input_shapes_[4], {1, 2}) && input_dtypes_[4] == nncase::dt_float16 &&
        shape_is(input_shapes_[5], {1, 2}) && input_dtypes_[5] == nncase::dt_float16;
    const bool output_ok = shape_is(output_shapes_[0], {1, static_cast<int>(kOutputFloats)}) &&
        output_dtypes_[0] == nncase::dt_float32;
    if (!inputs_ok || !output_ok) {
        throw std::runtime_error(
            "incompatible kmodel ABI; expected u8 images, f16 context inputs, and f32[1,2576] output");
    }
}

void SupercomboModel::reset_temporal_state()
{
    for (auto &frame : image_history_) frame.fill(0);
    for (auto &frame : big_image_history_) frame.fill(0);
    image_history_head_ = 0;
    model_context_.reset();
    desire_pulse_.fill(0.0f);
    previous_desire_.fill(0.0f);
}

void SupercomboModel::set_desire(int desire)
{
    current_desire_ = desire;
    desire_pulse_.fill(0.0f);
    for (int i = 1; i < static_cast<int>(desire_pulse_.size()); ++i) {
        const float current = i == desire ? 1.0f : 0.0f;
        desire_pulse_[i] = current - previous_desire_[i] > 0.99f ? current : 0.0f;
        previous_desire_[i] = current;
    }
}

void SupercomboModel::set_input_calibration(const float rpy[3])
{
    input_transform_.set_calibration(rpy[0], rpy[1], rpy[2]);
    big_input_transform_.set_calibration(rpy[0], rpy[1], rpy[2]);
}

bool SupercomboModel::run_frame_nv12(const uint8_t *nv12, int src_w, int src_h,
                                     std::vector<float> &raw_output)
{
    if (!nv12 || src_w <= 0 || src_h <= 0) return false;

    const bool profile = profile_enabled();
    const uint64_t t0 = profile ? now_ns() : 0;
    const size_t nv12_bytes = static_cast<size_t>(src_w) * static_cast<size_t>(src_h) * 3 / 2;
    nv12_cache_.resize(nv12_bytes);
    std::memcpy(nv12_cache_.data(), nv12, nv12_bytes);
    const uint64_t t1 = profile ? now_ns() : 0;

    if (!prepare_image_input(0, input_transform_, image_history_,
                             nv12_cache_.data(), src_w, src_h) ||
        !prepare_image_input(1, big_input_transform_, big_image_history_,
                             nv12_cache_.data(), src_w, src_h)) {
        return false;
    }
    const uint64_t t2 = profile ? now_ns() : 0;

    model_context_.prepare_feature_input(feature_input_.data(), feature_input_.size());
    model_context_.prepare_desire_input(desire_pulse_.data(), desire_pulse_.size(),
                                        desire_input_.data(), desire_input_.size());
    if (!write_input_f16(2, feature_input_.data(), feature_input_.size()) ||
        !write_input_f16(3, desire_input_.data(), desire_input_.size()) ||
        !write_input_f16(4, traffic_input_.data(), traffic_input_.size()) ||
        !write_input_f16(5, action_input_.data(), action_input_.size())) {
        return false;
    }
    const uint64_t t3 = profile ? now_ns() : 0;

    run();
    const uint64_t t4 = profile ? now_ns() : 0;
    if (!commit_image_history(0, image_history_) ||
        !commit_image_history(1, big_image_history_)) {
        return false;
    }
    image_history_head_ = (image_history_head_ + 1) % kImageHistoryFrames;
    const uint64_t t5 = profile ? now_ns() : 0;

    runtime_tensor output_tensor = get_output_tensor(0);
    hrt::sync(output_tensor, sync_op_t::sync_invalidate, true).expect("sync output failed");
    auto host_buffer = output_tensor.impl()->to_host().unwrap()->buffer().as_host().unwrap();
    auto mapped = std::move(host_buffer.map(map_access_::map_read).unwrap());
    const auto buffer = mapped.buffer();
    if (buffer.size() < kOutputFloats * sizeof(float)) {
        mapped.unmap().expect("unmap short output failed");
        reset_temporal_state();
        return false;
    }

    const float *output = reinterpret_cast<const float *>(buffer.data());
    raw_output.assign(output, output + kOutputFloats);
    mapped.unmap().expect("unmap output failed");
    if (!std::all_of(raw_output.begin(), raw_output.end(),
                     [](float value) { return std::isfinite(value); })) {
        raw_output.clear();
        reset_temporal_state();
        return false;
    }
    if (lane_plan_fusion_enabled_ && current_desire_ != 3 && current_desire_ != 4)
        fuse_lane_center_plan(raw_output);
    if (!model_context_.set_previous_hidden(raw_output.data() + kHiddenOffset,
                                            ModernModelContext::kFeatureCount)) {
        raw_output.clear();
        reset_temporal_state();
        return false;
    }

    if (profile) {
        const uint64_t t6 = now_ns();
        profile_stats().add(t1 - t0, t2 - t1, t3 - t2, t4 - t3,
                            t5 - t4, t6 - t5, t6 - t0);
    }
    return true;
}

bool SupercomboModel::prepare_image_input(size_t index, ModelInputTransform &transform,
                                          ImageHistory &history, const uint8_t *nv12,
                                          int src_w, int src_h)
{
    if (index >= input_tensors_.size() || shape_count(index) != kInputImageBytes)
        return false;

    auto host_buffer = input_tensors_[index].impl()->to_host().unwrap()->buffer().as_host().unwrap();
    auto mapped = std::move(host_buffer.map(map_access_::map_read_write).unwrap());
    auto buffer = mapped.buffer();
    if (buffer.size() < kInputImageBytes) {
        mapped.unmap().expect("unmap short image input failed");
        return false;
    }

    uint8_t *input = reinterpret_cast<uint8_t *>(buffer.data());
    std::memcpy(input, history[image_history_head_].data(), kYuv6Bytes);
    transform.nv12_to_yuv6_warped(nv12, src_w, src_h, input + kYuv6Bytes);
    mapped.unmap().expect("unmap image input failed");
    hrt::sync(input_tensors_[index], sync_op_t::sync_write_back, true)
        .expect("sync image input failed");
    return true;
}

bool SupercomboModel::commit_image_history(size_t index, ImageHistory &history)
{
    if (index >= input_tensors_.size() || shape_count(index) != kInputImageBytes)
        return false;

    auto host_buffer = input_tensors_[index].impl()->to_host().unwrap()->buffer().as_host().unwrap();
    auto mapped = std::move(host_buffer.map(map_access_::map_read).unwrap());
    const auto buffer = mapped.buffer();
    if (buffer.size() < kInputImageBytes) {
        mapped.unmap().expect("unmap short image history failed");
        return false;
    }
    std::memcpy(history[image_history_head_].data(),
                reinterpret_cast<const uint8_t *>(buffer.data()) + kYuv6Bytes,
                kYuv6Bytes);
    mapped.unmap().expect("unmap image history failed");
    return true;
}

bool SupercomboModel::write_input_bytes(size_t index, const void *data, size_t bytes)
{
    if (index >= input_tensors_.size() || !data) return false;
    auto host_buffer = input_tensors_[index].impl()->to_host().unwrap()->buffer().as_host().unwrap();
    auto mapped = std::move(host_buffer.map(map_access_::map_write).unwrap());
    auto buffer = mapped.buffer();
    if (buffer.size() < bytes) {
        mapped.unmap().expect("unmap short auxiliary input failed");
        return false;
    }
    std::memcpy(buffer.data(), data, bytes);
    mapped.unmap().expect("unmap auxiliary input failed");
    hrt::sync(input_tensors_[index], sync_op_t::sync_write_back, true)
        .expect("sync auxiliary input failed");
    return true;
}

bool SupercomboModel::write_input_f16(size_t index, const uint16_t *data, size_t count)
{
    if (index >= input_tensors_.size() || input_dtypes_[index] != nncase::dt_float16 ||
        shape_count(index) != count) {
        std::cerr << "float16 input " << index << " ABI mismatch" << std::endl;
        return false;
    }
    return write_input_bytes(index, data, count * sizeof(uint16_t));
}

size_t SupercomboModel::shape_count(size_t index) const
{
    if (index >= input_shapes_.size()) return 0;
    return std::accumulate(input_shapes_[index].begin(), input_shapes_[index].end(),
                           size_t{1}, std::multiplies<size_t>());
}
