#include "model_output.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

namespace {

template <typename T>
bool read_exact(std::ifstream &file, T *value)
{
    file.read(reinterpret_cast<char *>(value), sizeof(T));
    return file.gcount() == static_cast<std::streamsize>(sizeof(T));
}

} // namespace

int main(int argc, char *argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <SCODMP1 raw dump>\n";
        return 1;
    }

    std::ifstream file(argv[1], std::ios::binary);
    if (!file) {
        std::perror("open raw dump");
        return 1;
    }

    char magic[8]{};
    file.read(magic, sizeof(magic));
    if (file.gcount() != static_cast<std::streamsize>(sizeof(magic)) ||
        std::memcmp(magic, "SCODMP1", 7) != 0) {
        std::cerr << "bad raw dump magic\n";
        return 1;
    }

    uint32_t raw_size = 0;
    uint32_t frame_count = 0;
    if (!read_exact(file, &raw_size) || !read_exact(file, &frame_count) || raw_size == 0) {
        std::cerr << "bad raw dump header\n";
        return 1;
    }

    std::vector<float> raw(raw_size);
    file.read(reinterpret_cast<char *>(raw.data()), static_cast<std::streamsize>(raw.size() * sizeof(float)));
    if (file.gcount() != static_cast<std::streamsize>(raw.size() * sizeof(float))) {
        std::cerr << "short first frame\n";
        return 1;
    }

    const ParsedModelOutput parsed = ModelOutputParser::parse(raw);
    if (!parsed.valid || !parsed.plan.valid || !parsed.has_pose) {
        std::cerr << "parser did not produce required plan/pose outputs\n";
        return 1;
    }

    ParsedLeadPoint lead;
    const bool have_lead = parsed.leads.primary(0, 0.0f, &lead);
    std::cout << "raw_size=" << raw_size
              << " frames=" << frame_count
              << " plan_best=" << parsed.plan.best_index
              << " plan_prob=" << parsed.plan.probability
              << " lane0_prob=" << parsed.lanes[0].probability
              << " pose_vx=" << parsed.pose.trans[0]
              << " lead_valid=" << (have_lead ? 1 : 0);
    if (have_lead)
        std::cout << " lead_x=" << lead.x << " lead_y=" << lead.y;
    std::cout << "\n";
    return 0;
}
