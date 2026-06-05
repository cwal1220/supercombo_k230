#include "supercombo_model.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <numeric>

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
    double pack_ms = 0.0;
    double input_ms = 0.0;
    double run_ms = 0.0;
    double output_ms = 0.0;
    double total_ms = 0.0;

    void add(uint64_t preprocess_ns, uint64_t pack_ns, uint64_t input_ns,
             uint64_t run_ns, uint64_t output_ns, uint64_t total_ns)
    {
        ++count;
        preprocess_ms += ns_to_ms(preprocess_ns);
        pack_ms += ns_to_ms(pack_ns);
        input_ms += ns_to_ms(input_ns);
        run_ms += ns_to_ms(run_ns);
        output_ms += ns_to_ms(output_ns);
        total_ms += ns_to_ms(total_ns);

        if (count % 30 == 0) {
            const double denom = static_cast<double>(count);
            std::fprintf(stderr,
                         "\nprofile avg[%llu] ms: preprocess=%.3f pack=%.3f input=%.3f run=%.3f output=%.3f total=%.3f\n",
                         static_cast<unsigned long long>(count),
                         preprocess_ms / denom,
                         pack_ms / denom,
                         input_ms / denom,
                         run_ms / denom,
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

SupercomboModel::SupercomboModel(const char *kmodel_file, int debug_mode, const AppConfig &config)
    : AIBase(kmodel_file, "Supercombo", debug_mode),
      input_transform_(config),
      prev_yuv_(kYuv6Floats, 0.0f),
      current_yuv_(kYuv6Floats, 0.0f),
      input_imgs_(kInputImageFloats, 0.0f),
      big_input_imgs_(kInputImageFloats, 0.0f),
      desire_(8, 0.0f),
      traffic_convention_{1.0f, 0.0f},
      recurrent_state_(kRecurrentFloats, 0.0f)
{
    for (size_t i = 0; i < input_shapes_.size(); ++i)
        input_tensors_.push_back(get_input_tensor(i));
}

void SupercomboModel::reset_state()
{
    std::fill(prev_yuv_.begin(), prev_yuv_.end(), 0.0f);
    std::fill(recurrent_state_.begin(), recurrent_state_.end(), 0.0f);
}

bool SupercomboModel::run_frame_nv12(const uint8_t *nv12, int src_w, int src_h, std::vector<float> &raw_output)
{
    const bool profile = profile_enabled();
    const uint64_t t0 = profile ? now_ns() : 0;
    input_transform_.nv12_to_yuv6_warped(nv12, src_w, src_h, current_yuv_);
    const uint64_t t1 = profile ? now_ns() : 0;
    return run_current_yuv6(raw_output, profile, t0, t1);
}

bool SupercomboModel::run_current_yuv6(std::vector<float> &raw_output, bool profile, uint64_t t0, uint64_t t1)
{
    std::memcpy(input_imgs_.data(), prev_yuv_.data(), prev_yuv_.size() * sizeof(float));
    std::memcpy(input_imgs_.data() + prev_yuv_.size(), current_yuv_.data(), current_yuv_.size() * sizeof(float));
    prev_yuv_.swap(current_yuv_);
    const uint64_t t2 = profile ? now_ns() : 0;

    if (!write_input(0, input_imgs_.data(), input_imgs_.size())) return false;
    if (!write_input(1, big_input_imgs_.data(), big_input_imgs_.size())) return false;
    if (!write_input(2, desire_.data(), desire_.size())) return false;
    if (!write_input(3, traffic_convention_.data(), traffic_convention_.size())) return false;
    if (!write_input(4, recurrent_state_.data(), recurrent_state_.size())) return false;
    const uint64_t t3 = profile ? now_ns() : 0;

    run();
    const uint64_t t4 = profile ? now_ns() : 0;
    get_output();

    size_t total = 0;
    for (const auto &shape : output_shapes_)
        total += std::accumulate(shape.begin(), shape.end(), size_t{1}, std::multiplies<size_t>());

    raw_output.resize(total);
    size_t offset = 0;
    for (size_t i = 0; i < output_shapes_.size(); ++i) {
        const size_t count = std::accumulate(output_shapes_[i].begin(), output_shapes_[i].end(), size_t{1}, std::multiplies<size_t>());
        std::memcpy(raw_output.data() + offset, p_outputs_[i], count * sizeof(float));
        offset += count;
    }

    if (raw_output.size() >= kRecurrentFloats) {
        std::memcpy(recurrent_state_.data(), raw_output.data() + raw_output.size() - kRecurrentFloats,
                    kRecurrentFloats * sizeof(float));
    }

    if (profile) {
        const uint64_t t5 = now_ns();
        profile_stats().add(t1 - t0, t2 - t1, t3 - t2, t4 - t3, t5 - t4, t5 - t0);
    }

    return true;
}

void SupercomboModel::nv12_to_yuv6(const uint8_t *nv12, int src_w, int src_h, std::vector<float> &out)
{
    if (src_w < kModelW || src_h < kModelH) {
        std::fill(out.begin(), out.end(), 0.0f);
        return;
    }

    const uint8_t *y_plane = nv12;
    const uint8_t *uv_plane = nv12 + src_w * src_h;
    const int plane_size = kHalfW * kHalfH;
    float *y00_plane = out.data();
    float *y01_plane = y00_plane + plane_size;
    float *y10_plane = y01_plane + plane_size;
    float *y11_plane = y10_plane + plane_size;
    float *u_plane = y11_plane + plane_size;
    float *v_plane = u_plane + plane_size;

    if (src_w == kModelW && src_h == kModelH) {
        for (int y2 = 0; y2 < kHalfH; ++y2) {
            const int src_y0 = y2 * 2;
            const uint8_t *y0 = y_plane + src_y0 * src_w;
            const uint8_t *y1 = y0 + src_w;
            const uint8_t *uv = uv_plane + (src_y0 / 2) * src_w;
            float *dst_y00 = y00_plane + y2 * kHalfW;
            float *dst_y01 = y01_plane + y2 * kHalfW;
            float *dst_y10 = y10_plane + y2 * kHalfW;
            float *dst_y11 = y11_plane + y2 * kHalfW;
            float *dst_u = u_plane + y2 * kHalfW;
            float *dst_v = v_plane + y2 * kHalfW;

            for (int x2 = 0; x2 < kHalfW; ++x2) {
                dst_y00[x2] = static_cast<float>(y0[0]);
                dst_y01[x2] = static_cast<float>(y0[1]);
                dst_y10[x2] = static_cast<float>(y1[0]);
                dst_y11[x2] = static_cast<float>(y1[1]);
                dst_u[x2] = static_cast<float>(uv[0]);
                dst_v[x2] = static_cast<float>(uv[1]);
                y0 += 2;
                y1 += 2;
                uv += 2;
            }
        }
        return;
    }

    const int crop_h = std::min(src_h, src_w / 2);
    const int crop_y = std::max(0, (src_h - crop_h) / 2);

    for (int y2 = 0; y2 < kHalfH; ++y2) {
        float *dst_y00 = y00_plane + y2 * kHalfW;
        float *dst_y01 = y01_plane + y2 * kHalfW;
        float *dst_y10 = y10_plane + y2 * kHalfW;
        float *dst_y11 = y11_plane + y2 * kHalfW;
        float *dst_u = u_plane + y2 * kHalfW;
        float *dst_v = v_plane + y2 * kHalfW;
        float *dst_y[4] = {dst_y00, dst_y01, dst_y10, dst_y11};

        for (int x2 = 0; x2 < kHalfW; ++x2) {
            int sum_u = 0;
            int sum_v = 0;
            for (int dy = 0; dy < 2; ++dy) {
                for (int dx = 0; dx < 2; ++dx) {
                    const int mx = x2 * 2 + dx;
                    const int my = y2 * 2 + dy;
                    const int sx = std::min(src_w - 1, mx * src_w / kModelW);
                    const int sy = std::min(src_h - 1, crop_y + my * crop_h / kModelH);
                    const uint8_t yy = y_plane[sy * src_w + sx];
                    const int uv_x = sx & ~1;
                    const uint8_t *uv = uv_plane + (sy / 2) * src_w + uv_x;
                    dst_y[dy * 2 + dx][x2] = static_cast<float>(yy);
                    sum_u += uv[0];
                    sum_v += uv[1];
                }
            }
            dst_u[x2] = static_cast<float>(sum_u / 4);
            dst_v[x2] = static_cast<float>(sum_v / 4);
        }
    }
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
