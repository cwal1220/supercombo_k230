#ifndef IPC_CHANNELS_H
#define IPC_CHANNELS_H

/* /dev/shm 채널 구현: 최신값 채널, CAN 큐, NV12 프레임 링. 레이아웃은
 * ipc_messages.h에 있다. */

#include "ipc_messages.h"

#include <cstddef>
#include <cstdint>
#include <string>

/* shm_open한 이름 하나와 그 매핑. 세 채널이 열기·크기 조정·매핑·닫기 순서를 공유하고,
 * 크기 정책(만들 때 늘릴지, 붙을 때 얼마를 요구할지)만 각자 정한다. */
class ShmRegion {
public:
    ShmRegion() = default;
    ~ShmRegion() { close(); }
    ShmRegion(const ShmRegion &) = delete;
    ShmRegion &operator=(const ShmRegion &) = delete;

    bool open(const char *name, bool create);
    bool file_size(size_t *size) const;
    bool resize(size_t size);
    bool map(size_t size);
    void close();
    void *data() const { return map_; }
    size_t size() const { return size_; }

private:
    int fd_ = -1;
    void *map_ = nullptr;
    size_t size_ = 0;
};

class K230LatestChannel {
public:
    K230LatestChannel() = default;
    ~K230LatestChannel();

    bool open(const char *name, size_t payload_capacity, bool create);
    void close();
    bool publish(const void *payload, size_t payload_size);
    bool read(void *payload, size_t payload_capacity, uint64_t *seq = nullptr) const;
    bool read_new(uint64_t *last_seq, void *payload, size_t payload_capacity, int timeout_ms) const;
    bool valid() const { return header_ != nullptr; }

private:
    std::string name_;
    ShmRegion region_;
    K230IpcHeader *header_ = nullptr;
    uint8_t *payload_ = nullptr;
};

class K230CanQueue {
public:
    K230CanQueue() = default;
    ~K230CanQueue();

    bool open(const char *name, unsigned slot_count = kK230CanQueueSlots,
              bool create = true);
    void close();
    void reset();
    bool push(const K230CanBatch &batch);
    bool pop(K230CanBatch *batch);
    uint64_t depth() const;
    bool valid() const { return header_ != nullptr; }

private:
    std::string name_;
    ShmRegion region_;
    K230CanQueueHeader *header_ = nullptr;
    K230CanBatch *slots_ = nullptr;
};

class K230FrameRing {
public:
    K230FrameRing() = default;
    ~K230FrameRing();

    bool open(bool create, unsigned width = kK230AiWidth, unsigned height = kK230AiHeight,
              unsigned slots = kK230FrameSlots);
    void close();
    bool write_slot(unsigned index, uint64_t frame_id, const uint8_t *source,
                    size_t size);
    bool copy_slot(unsigned index, uint64_t frame_id, uint8_t *destination,
                   size_t size) const;
    /* NV12 슬롯을 Y/UV 목적지로 나눠 복사한다. 소비자가 이미 스트라이드가
     * 있는 버퍼(예: 인코더 입력)를 쥐고 있을 때 중간 복사를 없앤다. */
    bool copy_slot_planes(unsigned index, uint64_t frame_id, uint8_t *luma,
                          size_t luma_stride, uint8_t *chroma,
                          size_t chroma_stride) const;
    unsigned slot_count() const { return header_ ? header_->slot_count : 0; }
    unsigned frame_bytes() const { return header_ ? header_->frame_bytes : 0; }
    unsigned width() const { return header_ ? header_->width : 0; }
    unsigned height() const { return header_ ? header_->height : 0; }
    bool valid() const { return header_ != nullptr; }

private:
    ShmRegion region_;
    K230FrameRingHeader *header_ = nullptr;
    uint8_t *frames_ = nullptr;
};

#endif
