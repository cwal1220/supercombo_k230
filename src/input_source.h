#ifndef INPUT_SOURCE_H
#define INPUT_SOURCE_H

#include "app_config.h"
#include "v4l2-drm.h"

#include <cstdint>
#include <cstddef>
#include <fstream>
#include <string>
#include <vector>

struct Nv12Frame {
    unsigned width = 0;
    unsigned height = 0;
    std::vector<uint8_t> data;
};

/* SCNV12R1 리플레이 파일의 NV12 프레임 소스. k230_modeld의 헤드리스 재생용. */
class ReplayNv12Source {
public:
    explicit ReplayNv12Source(const std::string &path);

    bool read(Nv12Frame &frame);
    bool eof() const { return frames_read_ >= frame_count_; }
    unsigned frame_count() const { return frame_count_; }
    unsigned width() const { return width_; }
    unsigned height() const { return height_; }

private:
    void read_exact(char *dst, size_t size, const char *label);

    std::string path_;
    std::ifstream file_;
    unsigned width_ = 0;
    unsigned height_ = 0;
    unsigned frame_count_ = 0;
    size_t frame_bytes_ = 0;
    unsigned frames_read_ = 0;
};

/* v4l2 캡처의 NV12 프레임 소스. k230_camerad가 끝없이 읽으므로 eof가 없다. */
class LiveNv12Source {
public:
    LiveNv12Source(const AppConfig &config, int video_device);
    ~LiveNv12Source();

    // v4l2 컨텍스트를 소유하므로 복사하면 stop이 두 번 불린다.
    LiveNv12Source(const LiveNv12Source &) = delete;
    LiveNv12Source &operator=(const LiveNv12Source &) = delete;

    bool read(Nv12Frame &frame);

private:
    v4l2_drm_context context_{};
    bool started_ = false;
};

#endif
