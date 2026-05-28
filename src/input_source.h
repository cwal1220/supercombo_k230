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

class FrameSource {
public:
    virtual ~FrameSource() = default;
    virtual bool read(Nv12Frame &frame) = 0;
    virtual bool eof() const { return false; }
    virtual unsigned frame_count() const { return 0; }
};

class ReplayNv12Source : public FrameSource {
public:
    explicit ReplayNv12Source(const std::string &path);

    bool read(Nv12Frame &frame) override;
    bool eof() const override { return frames_read_ >= frame_count_; }
    unsigned frame_count() const override { return frame_count_; }

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

class LiveNv12Source : public FrameSource {
public:
    LiveNv12Source(const AppConfig &config, int video_device);
    ~LiveNv12Source() override;

    bool read(Nv12Frame &frame) override;

private:
    v4l2_drm_context context_{};
    bool started_ = false;
};

#endif
