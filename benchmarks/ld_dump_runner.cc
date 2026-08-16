/* LD kmodel 검증 러너: NV12 원시 프레임 파일을 순회하며 LdModel을 돌리고
 * 디코드된 라인(슬롯/색/패턴/폴리라인)을 텍스트로 덤프한다. 호스트에서
 * 프레임 위에 그려 색상 분류를 눈으로 확인하는 용도. */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "../src/ld_model.h"

int main(int argc, char *argv[]) {
    if (argc != 5) {
        std::fprintf(stderr, "usage: %s <kmodel> <nv12_file> <width> <height>\n", argv[0]);
        return 1;
    }
    const char *kmodel_path = argv[1];
    const char *nv12_path = argv[2];
    const unsigned width = static_cast<unsigned>(std::atoi(argv[3]));
    const unsigned height = static_cast<unsigned>(std::atoi(argv[4]));
    const size_t frame_bytes = static_cast<size_t>(width) * height * 3 / 2;

    FILE *fp = std::fopen(nv12_path, "rb");
    if (!fp) {
        std::fprintf(stderr, "cannot open %s\n", nv12_path);
        return 1;
    }

    LdModel model(kmodel_path, width, height);
    std::vector<uint8_t> frame(frame_bytes);
    unsigned index = 0;
    while (std::fread(frame.data(), 1, frame_bytes, fp) == frame_bytes) {
        LdModelTiming timing {};
        const auto lines = model.run(frame.data(), &timing);
        std::printf("FRAME %u lines=%zu infer_ms=%.2f\n", index, lines.size(), timing.infer_ms);
        for (const auto &line : lines) {
            std::printf("LINE kind=%d slot=%d color=%d pattern=%d double=%d btype=%d valid=%.3f conf=%.3f npts=%zu\n",
                        static_cast<int>(line.kind), line.slot,
                        static_cast<int>(line.marker_color),
                        static_cast<int>(line.marker_pattern),
                        line.raw_double_shape,
                        static_cast<int>(line.boundary_type),
                        line.validity, line.confidence, line.points.size());
            std::printf("PTS");
            for (const auto &pt : line.points) {
                std::printf(" %.4f,%.4f,%.2f", pt.x, pt.y, pt.confidence);
            }
            std::printf("\n");
        }
        ++index;
    }
    std::fclose(fp);
    std::fprintf(stderr, "done frames=%u\n", index);
    return 0;
}
