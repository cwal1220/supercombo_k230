#include "k230_ipc.h"

#include <sys/mman.h>
#include <unistd.h>

#include <cstdio>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

K230CanBatch batch_for(uint32_t value) {
  K230CanBatch batch;
  batch.timestamp_ns = value;
  batch.valid = 1;
  batch.count = 1;
  batch.frames[0].address = value;
  return batch;
}

}  // namespace

int main() {
  const std::string name = "/k230_can_queue_test_" + std::to_string(getpid());
  try {
    K230CanQueue producer;
    K230CanQueue consumer;
    require(producer.open(name.c_str(), 8, true), "open producer");
    producer.reset();
    require(consumer.open(name.c_str(), 8, false), "open consumer");

    for (uint32_t i = 0; i < 8; ++i) {
      require(producer.push(batch_for(i)), "fill queue");
    }
    require(producer.depth() == 8, "full queue depth");
    require(!producer.push(batch_for(8)), "full queue must not overwrite");

    for (uint32_t i = 0; i < 8; ++i) {
      K230CanBatch batch;
      require(consumer.pop(&batch), "drain queue");
      require(batch.valid && batch.timestamp_ns == i &&
                  batch.frames[0].address == i,
              "queue ordering");
    }
    require(!consumer.pop(nullptr), "null pop");
    K230CanBatch empty;
    require(!consumer.pop(&empty), "empty queue");

    require(producer.push(batch_for(41)), "push before producer reopen");
    producer.close();
    require(producer.open(name.c_str(), 8, true), "reopen producer");
    K230CanBatch after_reopen;
    require(consumer.pop(&after_reopen), "queue survives producer reopen");
    require(after_reopen.frames[0].address == 41,
            "producer reopen must preserve queued data");

    require(producer.push(batch_for(42)), "push before producer reset");
    producer.reset();
    require(!consumer.pop(&empty), "producer reset discards previous generation");

    for (uint32_t i = 0; i < 10000; ++i) {
      require(producer.push(batch_for(i)), "interleaved push");
      K230CanBatch batch;
      require(consumer.pop(&batch), "interleaved pop");
      require(batch.timestamp_ns == i, "interleaved ordering");
    }
    require(producer.depth() == 0, "empty queue depth");
    K230CanBatch fresh = batch_for(1);
    fresh.timestamp_ns = 1000;
    require(k230_can_batch_is_fresh(fresh, 1050, 100), "fresh batch");
    require(!k230_can_batch_is_fresh(fresh, 1101, 100), "stale batch");
    require(!k230_can_batch_is_fresh(fresh, 999, 100), "future batch");

    consumer.close();
    producer.close();
    shm_unlink(name.c_str());
    std::puts("K230_CAN_QUEUE_OK");
    return 0;
  } catch (const std::exception &error) {
    shm_unlink(name.c_str());
    std::fprintf(stderr, "check_k230_can_queue: %s\n", error.what());
    return 1;
  }
}
