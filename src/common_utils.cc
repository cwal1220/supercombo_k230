#include "common_utils.h"
#include "k230_ipc.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>

K230LatestChannel::~K230LatestChannel()
{
    close();
}

bool K230LatestChannel::open(const char *name, size_t payload_capacity, bool create)
{
    close();
    name_ = name ? name : "";
    const int flags = O_RDWR | (create ? O_CREAT : 0);
    fd_ = shm_open(name_.c_str(), flags, 0664);
    if (fd_ < 0) return false;

    map_size_ = sizeof(K230IpcHeader) + payload_capacity;
    if (create && ftruncate(fd_, static_cast<off_t>(map_size_)) != 0) {
        std::perror("ftruncate ipc channel");
        close();
        return false;
    }

    void *map = mmap(nullptr, map_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (map == MAP_FAILED) {
        std::perror("mmap ipc channel");
        close();
        return false;
    }

    header_ = static_cast<K230IpcHeader *>(map);
    payload_ = reinterpret_cast<uint8_t *>(header_) + sizeof(K230IpcHeader);
    if (create && (header_->magic != kK230IpcMagic ||
                   header_->version != kK230IpcVersion ||
                   header_->payload_capacity != payload_capacity)) {
        header_->magic = kK230IpcMagic;
        header_->version = kK230IpcVersion;
        header_->payload_capacity = static_cast<uint32_t>(payload_capacity);
        header_->reserved0 = 0;
        header_->seq.store(0, std::memory_order_release);
        header_->timestamp_ns.store(0, std::memory_order_release);
        header_->payload_size.store(0, std::memory_order_release);
        header_->reserved1 = 0;
        std::memset(payload_, 0, payload_capacity);
    }
    return header_->magic == kK230IpcMagic &&
        header_->version == kK230IpcVersion &&
        header_->payload_capacity >= payload_capacity;
}

void K230LatestChannel::close()
{
    if (header_) {
        munmap(header_, map_size_);
        header_ = nullptr;
        payload_ = nullptr;
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    map_size_ = 0;
}

bool K230LatestChannel::publish(const void *payload, size_t payload_size)
{
    if (!header_ || !payload || payload_size > header_->payload_capacity) return false;

    uint64_t seq = header_->seq.load(std::memory_order_acquire);
    if ((seq & 1ULL) != 0) ++seq;
    header_->seq.store(seq + 1, std::memory_order_release);
    std::memcpy(payload_, payload, payload_size);
    header_->payload_size.store(static_cast<uint32_t>(payload_size), std::memory_order_release);
    header_->timestamp_ns.store(k230_now_ns(), std::memory_order_release);
    header_->seq.store(seq + 2, std::memory_order_release);
    return true;
}

bool K230LatestChannel::read(void *payload, size_t payload_capacity, uint64_t *seq) const
{
    if (!header_ || !payload) return false;
    for (int attempt = 0; attempt < 4; ++attempt) {
        const uint64_t before = header_->seq.load(std::memory_order_acquire);
        if (before == 0 || (before & 1ULL) != 0) return false;
        const uint32_t payload_size = header_->payload_size.load(std::memory_order_acquire);
        if (payload_size == 0 || payload_size > payload_capacity) return false;
        std::memcpy(payload, payload_, payload_size);
        const uint64_t after = header_->seq.load(std::memory_order_acquire);
        if (before == after && (after & 1ULL) == 0) {
            if (seq) *seq = after;
            return true;
        }
    }
    return false;
}

bool K230LatestChannel::read_new(uint64_t *last_seq, void *payload,
                                 size_t payload_capacity, int timeout_ms) const
{
    const uint64_t start = k230_now_ns();
    const uint64_t timeout_ns = timeout_ms < 0
        ? UINT64_MAX
        : static_cast<uint64_t>(timeout_ms) * 1000000ULL;
    while (true) {
        uint64_t seq = 0;
        if (read(payload, payload_capacity, &seq) && (!last_seq || seq != *last_seq)) {
            if (last_seq) *last_seq = seq;
            return true;
        }
        if (timeout_ms == 0) return false;
        if (timeout_ms > 0 && k230_now_ns() - start >= timeout_ns) return false;
        usleep(1000);
    }
}
