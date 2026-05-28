#include "input_source.h"

#include <linux/videodev2.h>

#include <cstdio>
#include <cstring>
#include <stdexcept>

ReplayNv12Source::ReplayNv12Source(const std::string &path)
    : path_(path)
{
    file_.open(path_, std::ios::binary);
    if (!file_)
        throw std::runtime_error("open replay file failed: " + path_);

    char magic[8]{};
    read_exact(magic, sizeof(magic), "magic");
    if (std::memcmp(magic, "SCNV12R1", sizeof(magic)) != 0)
        throw std::runtime_error("bad replay magic: " + path_);

    read_exact(reinterpret_cast<char *>(&width_), sizeof(width_), "width");
    read_exact(reinterpret_cast<char *>(&height_), sizeof(height_), "height");
    read_exact(reinterpret_cast<char *>(&frame_count_), sizeof(frame_count_), "frame count");

    if (width_ == 0 || height_ == 0 || (width_ & 1) || (height_ & 1))
        throw std::runtime_error("bad replay dimensions: " + path_);
    frame_bytes_ = static_cast<size_t>(width_) * height_ * 3 / 2;
}

void ReplayNv12Source::read_exact(char *dst, size_t size, const char *label)
{
    file_.read(dst, static_cast<std::streamsize>(size));
    if (file_.gcount() != static_cast<std::streamsize>(size))
        throw std::runtime_error(std::string("short replay header read: ") + label + ": " + path_);
}

bool ReplayNv12Source::read(Nv12Frame &frame)
{
    if (eof()) return false;
    frame.width = width_;
    frame.height = height_;
    frame.data.resize(frame_bytes_);
    file_.read(reinterpret_cast<char *>(frame.data.data()), static_cast<std::streamsize>(frame.data.size()));
    if (file_.gcount() != static_cast<std::streamsize>(frame.data.size()))
        throw std::runtime_error("short replay frame read: " + path_);
    ++frames_read_;
    return true;
}

LiveNv12Source::LiveNv12Source(const AppConfig &config, int video_device)
{
    v4l2_drm_default_context(&context_);
    context_.device = video_device;
    context_.display = false;
    context_.width = config.nv12_width;
    context_.height = config.nv12_height;
    context_.video_format = V4L2_PIX_FMT_NV12;
    context_.crop_size.crop_en = 1;
    context_.crop_size.offset_x = config.nv12_crop_x;
    context_.crop_size.offset_y = config.nv12_crop_y;
    context_.crop_size.width = config.nv12_crop_width;
    context_.crop_size.height = config.nv12_crop_height;
    context_.buffer_num = 3;

    std::fprintf(stderr, "AI input %ux%u format=NV12 device=/dev/video%d",
                 context_.width, context_.height, static_cast<int>(context_.device));
    if (context_.crop_size.crop_en) {
        std::fprintf(stderr, " crop=%ux%u+%u+%u",
                     context_.crop_size.width, context_.crop_size.height,
                     context_.crop_size.offset_x, context_.crop_size.offset_y);
    }
    std::fprintf(stderr, "\n");

    if (v4l2_drm_setup(&context_, 1, nullptr))
        throw std::runtime_error("AI v4l2_drm_setup error");
    if (v4l2_drm_start(&context_))
        throw std::runtime_error("AI v4l2_drm_start error");
    started_ = true;
}

LiveNv12Source::~LiveNv12Source()
{
    if (started_)
        v4l2_drm_stop(&context_);
}

bool LiveNv12Source::read(Nv12Frame &frame)
{
    if (v4l2_drm_dump(&context_, 1000)) {
        std::perror("AI v4l2_drm_dump error");
        return false;
    }

    const size_t frame_bytes = static_cast<size_t>(context_.width) * context_.height * 3 / 2;
    const uint8_t *src = static_cast<const uint8_t *>(context_.buffers[context_.vbuffer.index].mmap);
    frame.width = context_.width;
    frame.height = context_.height;
    frame.data.assign(src, src + frame_bytes);
    v4l2_drm_dump_release(&context_);
    return true;
}
