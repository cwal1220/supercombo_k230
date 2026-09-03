// Stream raw decoded NV12 through the production fixed12 dual-camera warp.
// Build on the host; writes [medmodel (6,128,256), sbigmodel (6,128,256)] uint8.
#include "model_input_transform.h"
#include <iostream>
#include <stdexcept>
#include <vector>

int main(int argc, char **argv) {
    if (argc != 6) {
        std::cerr << "Usage: warp_recording width height roll pitch yaw < frames.nv12 > warped.bin\n";
        return 1;
    }
    try {
        const int w = std::stoi(argv[1]), h = std::stoi(argv[2]);
        if (w <= 0 || h <= 0 || w % 2 || h % 2) throw std::runtime_error("invalid dimensions");
        AppConfig config;
        config.input_warp_fx = default_input_warp_fx(w);
        config.input_warp_fy = default_input_warp_fy(h);
        config.input_warp_cx = default_input_warp_cx(w);
        config.input_warp_cy = default_input_warp_cy(h);
        config.manual_roll = std::stof(argv[3]);
        config.manual_pitch = std::stof(argv[4]);
        config.manual_yaw = std::stof(argv[5]);
        ModelInputTransform med(config, ModelFrame::MedModel), big(config, ModelFrame::SmallBigModel);
        std::vector<uint8_t> frame(static_cast<size_t>(w)*h*3/2), out(2*6*128*256);
        while (std::cin.read(reinterpret_cast<char *>(frame.data()), frame.size())) {
            med.nv12_to_yuv6_warped_scalar(frame.data(), w, h, out.data());
            big.nv12_to_yuv6_warped_scalar(frame.data(), w, h, out.data()+6*128*256);
            std::cout.write(reinterpret_cast<char *>(out.data()), out.size());
        }
        if (std::cin.gcount()) throw std::runtime_error("partial NV12 frame");
        if (!std::cout) throw std::runtime_error("failed to write warped frames");
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
