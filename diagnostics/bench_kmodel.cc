/* 아무 kmodel이나 읽어 0으로 채운 입력으로 돌리고 추론 1회의 벽시계 시간을 잰다. 후보 kmodel을
 * 배포 중인 supercombo와 실제 보드에서 비교할 때 쓴다(측정 전 파이프라인을 멈춰야 한다).
 * 사용: bench_kmodel [--iterations N] <model.kmodel>   (기본 100회) */

#include <nncase/runtime/interpreter.h>
#include <nncase/runtime/runtime_op_utility.h>
#include <nncase/runtime/util.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <numeric>
#include <vector>

using namespace nncase;
using namespace nncase::runtime;

int main(int argc, char **argv)
{
    const char *model_path = nullptr;
    int iterations = 100;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--iterations") == 0 && i + 1 < argc) {
            iterations = std::atoi(argv[++i]);
        } else if (std::strncmp(argv[i], "--", 2) != 0 && model_path == nullptr) {
            model_path = argv[i];
        } else {
            model_path = nullptr;
            break;
        }
    }
    if (model_path == nullptr) {
        std::fprintf(stderr, "usage: %s [--iterations N] <model.kmodel>\n", argv[0]);
        return 2;
    }
    if (iterations <= 0) {
        std::fprintf(stderr, "iterations must be positive\n");
        return 1;
    }

    interpreter interp;
    std::ifstream ifs(model_path, std::ios::binary);
    if (!ifs) {
        std::fprintf(stderr, "cannot open %s\n", model_path);
        return 1;
    }
    interp.load_model(ifs).expect("invalid kmodel");

    for (size_t i = 0; i < interp.inputs_size(); ++i) {
        auto desc = interp.input_desc(i);
        auto shape = interp.input_shape(i);
        auto tensor = host_runtime_tensor::create(desc.datatype, shape,
                                                  hrt::pool_shared)
                          .expect("cannot create input tensor");
        auto mapped = std::move(
            hrt::map(tensor, map_access_t::map_write).unwrap());
        std::memset(mapped.buffer().data(), 0, mapped.buffer().size_bytes());
        mapped.release();
        hrt::sync(tensor, sync_op_t::sync_write_back, true)
            .expect("sync input failed");
        interp.input_tensor(i, tensor).expect("cannot set input tensor");
        size_t count = std::accumulate(shape.begin(), shape.end(), size_t{1},
                                       std::multiplies<size_t>());
        std::fprintf(stderr, "input %zu: dtype=%d elems=%zu\n", i,
                     static_cast<int>(desc.datatype), count);
    }

    // 출력 텐서도 run() 전에 만들어 바인딩한다. run_kmodel_sequence와 같은
    // 방식이며, 런타임이 출력 바인딩을 요구하는 kmodel에서도 동작한다.
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

    for (int i = 0; i < 5; ++i)
        interp.run().expect("warmup run failed");

    std::vector<double> times_ms;
    times_ms.reserve(iterations);
    for (int i = 0; i < iterations; ++i) {
        const auto start = std::chrono::steady_clock::now();
        interp.run().expect("run failed");
        const auto end = std::chrono::steady_clock::now();
        times_ms.push_back(
            std::chrono::duration<double, std::milli>(end - start).count());
    }
    std::sort(times_ms.begin(), times_ms.end());
    const double mean = std::accumulate(times_ms.begin(), times_ms.end(), 0.0)
        / times_ms.size();
    std::printf("%s: iters=%d mean=%.2fms median=%.2fms min=%.2fms p95=%.2fms\n",
                model_path, iterations, mean,
                times_ms[times_ms.size() / 2], times_ms.front(),
                times_ms[static_cast<size_t>(times_ms.size() * 0.95)]);
    return 0;
}
