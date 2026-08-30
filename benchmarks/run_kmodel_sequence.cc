// Runs a kmodel over a recorded input sequence and dumps every output frame,
// so host fp32 references and board quantized results can be compared tick by
// tick. Input tensors are fully specified per tick, so no temporal state is
// kept here: quantization error cannot accumulate through this harness and the
// comparison stays open-loop and exact.
//
// Sequence file (little endian):
//   magic "K230MSQ1", u32 n_ticks, u32 n_inputs,
//   per input: u8 name_len, name, u32 dtype (0=uint8, 1=float32), u32 bytes,
//   then n_ticks x (all inputs, in header order).
//
// Usage: run_kmodel_sequence <model.kmodel> <seq.k230msq> <out.bin>

#include <nncase/runtime/interpreter.h>
#include <nncase/runtime/runtime_op_utility.h>
#include <nncase/runtime/util.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <numeric>
#include <string>
#include <vector>

using namespace nncase;
using namespace nncase::runtime;

namespace {

struct InputSpec {
    std::string name;
    uint32_t dtype = 0;
    uint32_t bytes = 0;
};

template <typename T>
bool read_pod(std::ifstream &file, T *value)
{
    return static_cast<bool>(file.read(reinterpret_cast<char *>(value), sizeof(T)));
}

} // namespace

int main(int argc, char **argv)
{
    if (argc < 4) {
        std::fprintf(stderr, "usage: %s <model.kmodel> <seq.k230msq> <out.bin>\n",
                     argv[0]);
        return 1;
    }

    std::fprintf(stderr, "[dbg] start\n");
    std::ifstream seq(argv[2], std::ios::binary);
    if (!seq) {
        std::fprintf(stderr, "cannot open %s\n", argv[2]);
        return 1;
    }
    char magic[8] = {};
    seq.read(magic, 8);
    if (std::memcmp(magic, "K230MSQ1", 8) != 0) {
        std::fprintf(stderr, "bad sequence magic\n");
        return 1;
    }
    std::fprintf(stderr, "[dbg] magic ok\n");
    uint32_t tick_count = 0;
    uint32_t input_count = 0;
    read_pod(seq, &tick_count);
    read_pod(seq, &input_count);

    std::vector<InputSpec> specs(input_count);
    for (auto &spec : specs) {
        uint8_t name_len = 0;
        read_pod(seq, &name_len);
        spec.name.resize(name_len);
        seq.read(spec.name.data(), name_len);
        read_pod(seq, &spec.dtype);
        read_pod(seq, &spec.bytes);
    }

    interpreter interp;
    std::ifstream model_file(argv[1], std::ios::binary);
    if (!model_file) {
        std::fprintf(stderr, "cannot open %s\n", argv[1]);
        return 1;
    }
    std::fprintf(stderr, "[dbg] header: ticks=%u inputs=%u\n", tick_count, input_count);
    interp.load_model(model_file).expect("invalid kmodel");
    std::fprintf(stderr, "[dbg] model loaded, inputs=%zu outputs=%zu\n", interp.inputs_size(), interp.outputs_size());

    if (interp.inputs_size() != input_count) {
        std::fprintf(stderr, "model has %zu inputs, sequence has %u\n",
                     interp.inputs_size(), input_count);
        return 1;
    }

    // The kmodel's input order does not have to match the sequence header
    // order, and the kmodel carries no input names, so pair them by byte size:
    // every input of this model has a distinct size.
    std::vector<runtime_tensor> tensors;
    std::vector<size_t> spec_for_input(interp.inputs_size());
    std::vector<bool> spec_used(specs.size(), false);
    for (size_t i = 0; i < interp.inputs_size(); ++i) {
        auto desc = interp.input_desc(i);
        auto shape = interp.input_shape(i);
        auto tensor = host_runtime_tensor::create(desc.datatype, shape,
                                                  hrt::pool_shared)
                          .expect("cannot create input tensor");
        interp.input_tensor(i, tensor).expect("cannot set input tensor");
        tensors.push_back(tensor);

        const size_t elems = std::accumulate(shape.begin(), shape.end(),
                                             size_t{1},
                                             std::multiplies<size_t>());
        const size_t elem_bytes = get_bytes(desc.datatype);
        const size_t bytes = elems * elem_bytes;
        size_t match = specs.size();
        for (size_t s = 0; s < specs.size(); ++s) {
            if (!spec_used[s] && specs[s].bytes == bytes) {
                match = s;
                break;
            }
        }
        if (match == specs.size()) {
            std::fprintf(stderr, "input %zu: no sequence entry of %zu bytes\n",
                         i, bytes);
            return 1;
        }
        spec_used[match] = true;
        spec_for_input[i] = match;
        std::fprintf(stderr, "input %zu <- %s (%zu bytes)\n", i,
                     specs[match].name.c_str(), bytes);
    }

    // outputs must be created and bound before run(), like AIBase does
    std::vector<runtime_tensor> out_tensors;
    for (size_t i = 0; i < interp.outputs_size(); ++i) {
        auto desc = interp.output_desc(i);
        auto shape = interp.output_shape(i);
        auto tensor = host_runtime_tensor::create(desc.datatype, shape,
                                                  hrt::pool_shared)
                          .expect("cannot create output tensor");
        interp.output_tensor(i, tensor).expect("cannot set output tensor");
        out_tensors.push_back(tensor);
    }

    std::ofstream out(argv[3], std::ios::binary);
    if (!out) {
        std::fprintf(stderr, "cannot write %s\n", argv[3]);
        return 1;
    }

    std::vector<std::vector<uint8_t>> tick_data(specs.size());
    double total_ms = 0.0;
    for (uint32_t tick = 0; tick < tick_count; ++tick) {
        for (size_t s = 0; s < specs.size(); ++s) {
            tick_data[s].resize(specs[s].bytes);
            if (!seq.read(reinterpret_cast<char *>(tick_data[s].data()),
                          specs[s].bytes)) {
                std::fprintf(stderr, "sequence truncated at tick %u\n", tick);
                return 1;
            }
        }
        for (size_t i = 0; i < tensors.size(); ++i) {
            const auto &src = tick_data[spec_for_input[i]];
            auto mapped = std::move(
                hrt::map(tensors[i], map_access_t::map_write).unwrap());
            std::memcpy(mapped.buffer().data(), src.data(), src.size());
            mapped.release();
            hrt::sync(tensors[i], sync_op_t::sync_write_back, true)
                .expect("sync input failed");
        }

        const auto start = std::chrono::steady_clock::now();
        interp.run().expect("run failed");
        total_ms += std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - start).count();

        hrt::sync(out_tensors[0], sync_op_t::sync_invalidate, true)
            .expect("sync output failed");
        auto span = out_tensors[0].impl()->to_host().unwrap()->buffer()
                        .as_host().unwrap().map(map_access_t::map_read)
                        .unwrap().buffer();
        out.write(reinterpret_cast<const char *>(span.data()),
                  static_cast<std::streamsize>(span.size_bytes()));
    }
    std::printf("%s: %u ticks, mean %.2f ms/tick, outputs -> %s\n", argv[1],
                tick_count, total_ms / tick_count, argv[3]);
    return 0;
}
