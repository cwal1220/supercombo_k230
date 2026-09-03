#include <nncase/runtime/interpreter.h>
#include <nncase/runtime/runtime_op_utility.h>
#include <nncase/runtime/util.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

using namespace nncase;
using namespace nncase::runtime;

namespace {

constexpr uint32_t kModeFloatYuv6 = 0;
constexpr uint32_t kModeNv12Both = 1;
constexpr uint32_t kModeUint8Yuv6 = 2;
// Same uint8 images plus an explicit reset flag and saved FP32 initial state.
constexpr uint32_t kModeUint8State = 3;
constexpr size_t kYuv6InputBytes = 12 * 128 * 256 * sizeof(float);
constexpr size_t kYuv6Uint8InputBytes = 12 * 128 * 256;
constexpr size_t kNv12InputBytes = 384 * 512;
constexpr size_t kDesireBytes = 8 * sizeof(float);
constexpr size_t kTrafficBytes = 2 * sizeof(float);
constexpr size_t kRecurrentFloats = 512;

struct SequenceHeader {
    char magic[8];
    uint32_t mode;
    uint32_t frames;
};

size_t shape_count(const dims_t &shape)
{
    return std::accumulate(shape.begin(), shape.end(), size_t{1}, std::multiplies<size_t>());
}

void read_exact(std::ifstream &stream, void *data, size_t bytes)
{
    stream.read(reinterpret_cast<char *>(data), static_cast<std::streamsize>(bytes));
    if (stream.gcount() != static_cast<std::streamsize>(bytes))
        throw std::runtime_error("short sequence input read");
}

void write_input(interpreter &interp, size_t index, const void *data, size_t bytes)
{
    auto tensor = interp.input_tensor(index).expect("cannot get input tensor");
    auto buf = tensor.impl()->to_host().unwrap()->buffer().as_host().unwrap()
                   .map(map_access_::map_write).unwrap().buffer();
    if (buf.size() != bytes) {
        throw std::runtime_error("input byte size mismatch at index " + std::to_string(index) +
                                 ": tensor=" + std::to_string(buf.size()) +
                                 " data=" + std::to_string(bytes));
    }
    std::memcpy(reinterpret_cast<char *>(buf.data()), data, bytes);
    hrt::sync(tensor, sync_op_t::sync_write_back, true).expect("sync write_back failed");
}

std::vector<float> read_outputs(interpreter &interp)
{
    std::vector<float> raw;
    for (size_t i = 0; i < interp.outputs_size(); ++i) {
        auto desc = interp.output_desc(i);
        if (desc.datatype != dt_float32)
            throw std::runtime_error("non-float32 output is not supported");
        const size_t count = shape_count(interp.output_shape(i));
        auto out = interp.output_tensor(i).expect("cannot get output tensor");
        auto buf = out.impl()->to_host().unwrap()->buffer().as_host().unwrap()
                       .map(map_access_::map_read).unwrap().buffer();
        const float *ptr = reinterpret_cast<const float *>(buf.data());
        raw.insert(raw.end(), ptr, ptr + count);
    }
    return raw;
}

void init_tensors(interpreter &interp)
{
    for (size_t i = 0; i < interp.inputs_size(); ++i) {
        auto desc = interp.input_desc(i);
        auto tensor = host_runtime_tensor::create(desc.datatype, interp.input_shape(i), hrt::pool_shared)
                          .expect("cannot create input tensor");
        interp.input_tensor(i, tensor).expect("cannot set input tensor");
    }
    for (size_t i = 0; i < interp.outputs_size(); ++i) {
        auto desc = interp.output_desc(i);
        auto tensor = host_runtime_tensor::create(desc.datatype, interp.output_shape(i), hrt::pool_shared)
                          .expect("cannot create output tensor");
        interp.output_tensor(i, tensor).expect("cannot set output tensor");
    }
}

void write_dump_header(std::ofstream &out, uint32_t raw_size, uint32_t frames)
{
    const char magic[8] = {'S', 'C', 'O', 'D', 'M', 'P', '1', '\0'};
    out.write(magic, sizeof(magic));
    out.write(reinterpret_cast<const char *>(&raw_size), sizeof(raw_size));
    out.write(reinterpret_cast<const char *>(&frames), sizeof(frames));
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <model.kmodel> <sequence.bin> <raw_out.bin>\n";
        return 1;
    }

    try {
        std::ifstream seq(argv[2], std::ios::binary);
        if (!seq) throw std::runtime_error("cannot open sequence input");
        SequenceHeader header{};
        read_exact(seq, &header, sizeof(header));
        if (std::memcmp(header.magic, "SCSEQ1\0\0", 8) != 0)
            throw std::runtime_error("bad sequence magic");
        if (header.mode != kModeFloatYuv6 && header.mode != kModeNv12Both &&
            header.mode != kModeUint8Yuv6 && header.mode != kModeUint8State)
            throw std::runtime_error("bad sequence mode");
        if (header.frames == 0) throw std::runtime_error("empty sequence");

        std::ifstream model_stream(argv[1], std::ios::binary);
        if (!model_stream) throw std::runtime_error("cannot open kmodel");
        interpreter interp;
        interp.load_model(model_stream).expect("Invalid kmodel");
        init_tensors(interp);

        const size_t expected_inputs = header.mode == kModeNv12Both ? 7 : 5;
        if (interp.inputs_size() != expected_inputs) {
            throw std::runtime_error("input count mismatch: model=" + std::to_string(interp.inputs_size()) +
                                     " seq=" + std::to_string(expected_inputs));
        }

        std::vector<uint8_t> buf0(std::max(kYuv6InputBytes, kNv12InputBytes));
        std::vector<uint8_t> buf1(std::max(kYuv6InputBytes, kNv12InputBytes));
        std::vector<uint8_t> buf2(kNv12InputBytes);
        std::vector<uint8_t> buf3(kNv12InputBytes);
        std::vector<float> desire(8);
        std::vector<float> traffic(2);
        std::vector<float> recurrent(kRecurrentFloats, 0.0f);
        std::vector<float> last_raw;

        std::ofstream raw_out(argv[3], std::ios::binary);
        if (!raw_out) throw std::runtime_error("cannot open raw output");
        uint32_t written = 0;
        uint32_t raw_size = 0;
        raw_out.seekp(16, std::ios::beg);

        std::vector<double> run_ms;
        run_ms.reserve(header.frames);
        for (uint32_t frame = 0; frame < header.frames; ++frame) {
            if (header.mode == kModeNv12Both) {
                read_exact(seq, buf0.data(), kNv12InputBytes);
                read_exact(seq, buf1.data(), kNv12InputBytes);
                read_exact(seq, buf2.data(), kNv12InputBytes);
                read_exact(seq, buf3.data(), kNv12InputBytes);
                read_exact(seq, desire.data(), kDesireBytes);
                read_exact(seq, traffic.data(), kTrafficBytes);
                write_input(interp, 0, buf0.data(), kNv12InputBytes);
                write_input(interp, 1, buf1.data(), kNv12InputBytes);
                write_input(interp, 2, buf2.data(), kNv12InputBytes);
                write_input(interp, 3, buf3.data(), kNv12InputBytes);
                write_input(interp, 4, desire.data(), kDesireBytes);
                write_input(interp, 5, traffic.data(), kTrafficBytes);
                write_input(interp, 6, recurrent.data(), recurrent.size() * sizeof(float));
            } else if (header.mode == kModeUint8Yuv6 || header.mode == kModeUint8State) {
                read_exact(seq, buf0.data(), kYuv6Uint8InputBytes);
                read_exact(seq, buf1.data(), kYuv6Uint8InputBytes);
                read_exact(seq, desire.data(), kDesireBytes);
                read_exact(seq, traffic.data(), kTrafficBytes);
                if (header.mode == kModeUint8State) {
                    uint32_t reset = 0;
                    std::vector<float> saved_state(kRecurrentFloats);
                    read_exact(seq, &reset, sizeof(reset));
                    read_exact(seq, saved_state.data(), saved_state.size() * sizeof(float));
                    if (reset > 1) throw std::runtime_error("bad state reset flag");
                    if (reset) recurrent = std::move(saved_state);
                }
                write_input(interp, 0, buf0.data(), kYuv6Uint8InputBytes);
                write_input(interp, 1, buf1.data(), kYuv6Uint8InputBytes);
                write_input(interp, 2, desire.data(), kDesireBytes);
                write_input(interp, 3, traffic.data(), kTrafficBytes);
                write_input(interp, 4, recurrent.data(), recurrent.size() * sizeof(float));
            } else {
                read_exact(seq, buf0.data(), kYuv6InputBytes);
                read_exact(seq, buf1.data(), kYuv6InputBytes);
                read_exact(seq, desire.data(), kDesireBytes);
                read_exact(seq, traffic.data(), kTrafficBytes);
                write_input(interp, 0, buf0.data(), kYuv6InputBytes);
                write_input(interp, 1, buf1.data(), kYuv6InputBytes);
                write_input(interp, 2, desire.data(), kDesireBytes);
                write_input(interp, 3, traffic.data(), kTrafficBytes);
                write_input(interp, 4, recurrent.data(), recurrent.size() * sizeof(float));
            }

            const auto t0 = std::chrono::steady_clock::now();
            interp.run().expect("error occurred in running model");
            const auto t1 = std::chrono::steady_clock::now();
            run_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());

            last_raw = read_outputs(interp);
            if (last_raw.size() >= recurrent.size()) {
                std::memcpy(recurrent.data(), last_raw.data() + last_raw.size() - recurrent.size(),
                            recurrent.size() * sizeof(float));
            }
            if (raw_size == 0) raw_size = static_cast<uint32_t>(last_raw.size());
            if (last_raw.size() != raw_size)
                throw std::runtime_error("raw output size changed");
            raw_out.write(reinterpret_cast<const char *>(last_raw.data()), last_raw.size() * sizeof(float));
            ++written;
            if (frame % 20 == 0)
                std::cerr << "ran " << (frame + 1) << "/" << header.frames << "\n";
        }

        raw_out.seekp(0, std::ios::beg);
        write_dump_header(raw_out, raw_size, written);

        const double mean = std::accumulate(run_ms.begin(), run_ms.end(), 0.0) / run_ms.size();
        std::sort(run_ms.begin(), run_ms.end());
        const double p50 = run_ms[run_ms.size() / 2];
        const double p95 = run_ms[std::min(run_ms.size() - 1, static_cast<size_t>(run_ms.size() * 95 / 100))];
        std::cerr << "frames=" << written << " raw_size=" << raw_size
                  << " run_mean_ms=" << mean << " run_p50_ms=" << p50
                  << " run_p95_ms=" << p95 << "\n";
    } catch (const std::exception &e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
