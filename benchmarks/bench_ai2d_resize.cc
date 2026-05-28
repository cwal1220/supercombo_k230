#include <nncase/functional/ai2d/ai2d_builder.h>
#include <nncase/runtime/runtime_op_utility.h>
#include <nncase/runtime/runtime_tensor.h>

#include <chrono>
#include <cstdio>
#include <cstring>

using namespace nncase;
using namespace nncase::runtime;
using namespace nncase::runtime::k230;
using namespace nncase::F::k230;

static uint64_t now_ns()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

int main()
{
    dims_t input_shape{1, 3, 1080, 1920};
    dims_t output_shape{1, 3, 256, 512};

    ai2d_datatype_t dtype{};
    dtype.src_format = ai2d_format::NCHW_FMT;
    dtype.dst_format = ai2d_format::NCHW_FMT;
    dtype.src_type = dt_uint8;
    dtype.dst_type = dt_uint8;
    dtype.src_loc = ai2d_data_loc::ddr;
    dtype.dst_loc = ai2d_data_loc::ddr;

    ai2d_crop_param_t crop{};
    crop.crop_flag = true;
    crop.start_x = 0;
    crop.start_y = 60;
    crop.width = 1920;
    crop.height = 960;

    ai2d_shift_param_t shift{};
    ai2d_pad_param_t pad{};
    pad.paddings = paddings_t{padding::zero(), padding::zero(), padding::zero(), padding::zero()};
    pad.pad_val = {0, 0, 0};

    ai2d_resize_param_t resize{};
    resize.resize_flag = true;
    resize.interp_method = ai2d_interp_method::tf_nearest;
    resize.interp_mode = ai2d_interp_mode::none;

    ai2d_affine_param_t affine{};

    auto builder = ai2d_builder::create(input_shape, output_shape, dtype, crop, shift, pad, resize, affine);
    if (!builder) {
        std::fprintf(stderr, "ai2d_builder::create failed\n");
        return 1;
    }
    builder->build_schedule().expect("ai2d build_schedule failed");

    auto input = host_runtime_tensor::create(dt_uint8, input_shape, hrt::pool_shared).expect("create input");
    auto output = host_runtime_tensor::create(dt_uint8, output_shape, hrt::pool_shared).expect("create output");

    auto in_buf = input.impl()->to_host().unwrap()->buffer().as_host().unwrap()
                      .map(map_access_::map_write).unwrap().buffer();
    std::memset(in_buf.data(), 127, in_buf.size());
    hrt::sync(input, sync_op_t::sync_write_back, true).expect("sync input");

    for (int i = 0; i < 5; ++i)
        builder->invoke(input, output).expect("ai2d warmup invoke failed");

    const int runs = 100;
    const uint64_t t0 = now_ns();
    for (int i = 0; i < runs; ++i)
        builder->invoke(input, output).expect("ai2d invoke failed");
    const uint64_t t1 = now_ns();

    std::printf("ai2d crop+resize avg_ms=%.3f runs=%d\n", (t1 - t0) / 1000000.0 / runs, runs);
    return 0;
}
