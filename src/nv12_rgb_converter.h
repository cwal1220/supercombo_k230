#pragma once

#include <cstdint>
#include <vector>

/* NV12 프레임의 행 구간 [row_begin, row_end)만 packed RGB24로 변환한다.
 * k230_adas의 BT.601 limited-range RVV 변환기를 이식하고 출력 순서를
 * BGR에서 RGB로 바꿨다(LD kmodel을 RGB 직입력으로 재컴파일했기 때문).
 * 행 경계는 NV12 UV 공유 때문에 짝수로 정렬되어야 한다. */
class Nv12RgbConverter {
public:
    // 변환할 소스 크기와 행 구간을 설정한다.
    void open(unsigned width, unsigned height, unsigned row_begin, unsigned row_end);

    // nv12에서 설정된 행 구간을 RGB24로 변환해 내부 버퍼에 담는다.
    void convert(const uint8_t *nv12);

    const uint8_t *rgb() const { return rgb_.data(); }
    unsigned rgb_width() const { return width_; }
    unsigned rgb_height() const { return row_end_ - row_begin_; }
    size_t rgb_bytes() const { return rgb_.size(); }

private:
    unsigned width_ = 0;
    unsigned height_ = 0;
    unsigned row_begin_ = 0;
    unsigned row_end_ = 0;
    std::vector<uint8_t> rgb_;
};
