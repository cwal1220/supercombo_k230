#include "k230_ipc.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace {

size_t queue_map_size(unsigned slot_count) {
  return sizeof(K230CanQueueHeader) +
      static_cast<size_t>(slot_count) * sizeof(K230CanBatch);
}

}  // namespace

bool k230_can_batch_is_fresh(const K230CanBatch &batch, uint64_t now_ns,
                             uint64_t max_age_ns) {
  return batch.valid && batch.timestamp_ns != 0 &&
      now_ns >= batch.timestamp_ns &&
      now_ns - batch.timestamp_ns <= max_age_ns;
}

K230CanQueue::~K230CanQueue() {
  close();
}

bool K230CanQueue::open(const char *name, unsigned slot_count, bool create) {
  close();
  if (!name || name[0] == '\0' || slot_count == 0) return false;

  name_ = name;
  fd_ = shm_open(name_.c_str(), O_RDWR | (create ? O_CREAT : 0), 0664);
  if (fd_ < 0) {
    std::perror("shm_open CAN queue");
    return false;
  }

  map_size_ = queue_map_size(slot_count);
  if (create) {
    struct stat st {};
    if (fstat(fd_, &st) != 0) {
      std::perror("fstat CAN queue");
      close();
      return false;
    }
    if (static_cast<size_t>(st.st_size) < map_size_ &&
        ftruncate(fd_, static_cast<off_t>(map_size_)) != 0) {
      std::fprintf(stderr,
                   "CAN queue resize failed name=%s actual=%lld expected=%zu\n",
                   name_.c_str(), static_cast<long long>(st.st_size), map_size_);
      std::perror("ftruncate CAN queue");
      close();
      return false;
    }
  } else {
    struct stat st {};
    if (fstat(fd_, &st) != 0 || static_cast<size_t>(st.st_size) < map_size_) {
      std::fprintf(stderr,
                   "CAN queue size mismatch name=%s actual=%lld expected=%zu\n",
                   name_.c_str(), static_cast<long long>(st.st_size), map_size_);
      close();
      return false;
    }
  }

  void *map = mmap(nullptr, map_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  if (map == MAP_FAILED) {
    std::perror("mmap CAN queue");
    close();
    return false;
  }

  header_ = static_cast<K230CanQueueHeader *>(map);
  slots_ = reinterpret_cast<K230CanBatch *>(
      reinterpret_cast<uint8_t *>(map) + sizeof(K230CanQueueHeader));
  if (create && (header_->magic != kK230CanQueueMagic ||
                 header_->version != kK230CanQueueVersion ||
                 header_->slot_count != slot_count)) {
    std::memset(map, 0, map_size_);
    header_->magic = kK230CanQueueMagic;
    header_->version = kK230CanQueueVersion;
    header_->slot_count = slot_count;
    header_->write_seq.store(0, std::memory_order_release);
    header_->read_seq.store(0, std::memory_order_release);
  }

  if (header_->magic != kK230CanQueueMagic ||
      header_->version != kK230CanQueueVersion ||
      header_->slot_count != slot_count) {
    std::fprintf(stderr,
                 "CAN queue header mismatch name=%s magic=0x%x version=%u slots=%u\n",
                 name_.c_str(), header_->magic, header_->version,
                 header_->slot_count);
    close();
    return false;
  }
  return true;
}

void K230CanQueue::close() {
  if (header_) {
    munmap(header_, map_size_);
    header_ = nullptr;
    slots_ = nullptr;
  }
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  map_size_ = 0;
}

void K230CanQueue::reset() {
  if (!header_) return;
  header_->read_seq.store(0, std::memory_order_release);
  header_->write_seq.store(0, std::memory_order_release);
  for (unsigned i = 0; i < header_->slot_count; ++i) {
    slots_[i] = K230CanBatch{};
  }
}

bool K230CanQueue::push(const K230CanBatch &batch) {
  if (!header_ || !slots_) return false;
  const uint64_t write_seq = header_->write_seq.load(std::memory_order_relaxed);
  const uint64_t read_seq = header_->read_seq.load(std::memory_order_acquire);
  if (write_seq - read_seq >= header_->slot_count) return false;

  slots_[write_seq % header_->slot_count] = batch;
  header_->write_seq.store(write_seq + 1, std::memory_order_release);
  return true;
}

bool K230CanQueue::pop(K230CanBatch *batch) {
  if (!header_ || !slots_ || !batch) return false;
  const uint64_t read_seq = header_->read_seq.load(std::memory_order_relaxed);
  const uint64_t write_seq = header_->write_seq.load(std::memory_order_acquire);
  if (read_seq == write_seq) return false;
  if (write_seq < read_seq || write_seq - read_seq > header_->slot_count) {
    return false;
  }

  *batch = slots_[read_seq % header_->slot_count];
  header_->read_seq.store(read_seq + 1, std::memory_order_release);
  return true;
}

uint64_t K230CanQueue::depth() const {
  if (!header_) return 0;
  const uint64_t write_seq = header_->write_seq.load(std::memory_order_acquire);
  const uint64_t read_seq = header_->read_seq.load(std::memory_order_acquire);
  if (write_seq < read_seq) return 0;
  return std::min<uint64_t>(write_seq - read_seq, header_->slot_count);
}
