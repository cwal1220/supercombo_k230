/* K230CanQueue: 공유 메모리 CAN 링 큐. 가득 참, 순서, 생산자 재열기·reset, 신선도 판정. */
#include "ipc_channels.h"

#include <gtest/gtest.h>

#include <sys/mman.h>
#include <unistd.h>

#include <string>

namespace {

K230CanBatch batch_for(uint32_t value) {
  K230CanBatch batch;
  batch.timestamp_ns = value;
  batch.valid = 1;
  batch.count = 1;
  batch.frames[0].address = value;
  return batch;
}

TEST(CanQueue, SharedMemoryQueue) {
  const std::string name = "/k230_can_queue_test_" + std::to_string(getpid());
  // ASSERT가 중간에 빠져나가도 공유 메모리를 지운다.
  struct Unlink {
    const std::string &name;
    ~Unlink() { shm_unlink(name.c_str()); }
  } cleanup{name};
  K230CanQueue producer;
  K230CanQueue consumer;
  ASSERT_TRUE(producer.open(name.c_str(), 8, true)) << "생산자 열기";
  producer.reset();
  ASSERT_TRUE(consumer.open(name.c_str(), 8, false)) << "소비자 열기";

  for (uint32_t i = 0; i < 8; ++i) {
    ASSERT_TRUE(producer.push(batch_for(i))) << "큐 채우기";
  }
  ASSERT_EQ(producer.depth(), 8) << "가득 찬 큐의 깊이";
  ASSERT_FALSE(producer.push(batch_for(8))) << "가득 찬 큐는 덮어쓰지 않는다";

  for (uint32_t i = 0; i < 8; ++i) {
    K230CanBatch batch;
    ASSERT_TRUE(consumer.pop(&batch)) << "큐 비우기";
    // 넣은 순서대로 나온다
    ASSERT_TRUE(batch.valid);
    ASSERT_EQ(batch.timestamp_ns, i);
    ASSERT_EQ(batch.frames[0].address, i);
  }
  ASSERT_FALSE(consumer.pop(nullptr)) << "null로 꺼내기";
  K230CanBatch empty;
  ASSERT_FALSE(consumer.pop(&empty)) << "빈 큐";

  ASSERT_TRUE(producer.push(batch_for(41))) << "생산자를 다시 열기 전에 넣기";
  producer.close();
  ASSERT_TRUE(producer.open(name.c_str(), 8, true)) << "생산자 다시 열기";
  K230CanBatch after_reopen;
  ASSERT_TRUE(consumer.pop(&after_reopen)) << "생산자를 다시 열어도 큐가 남는다";
  ASSERT_EQ(after_reopen.frames[0].address, 41) << "다시 연 생산자는 쌓인 데이터를 보존한다";

  ASSERT_TRUE(producer.push(batch_for(42))) << "생산자 reset 전에 넣기";
  producer.reset();
  ASSERT_FALSE(consumer.pop(&empty)) << "생산자 reset은 이전 세대를 버린다";

  for (uint32_t i = 0; i < 10000; ++i) {
    ASSERT_TRUE(producer.push(batch_for(i))) << "번갈아 넣기";
    K230CanBatch batch;
    ASSERT_TRUE(consumer.pop(&batch)) << "번갈아 꺼내기";
    ASSERT_EQ(batch.timestamp_ns, i) << "번갈아 넣고 꺼내도 순서가 유지된다";
  }
  ASSERT_EQ(producer.depth(), 0) << "빈 큐의 깊이";
  K230CanBatch fresh = batch_for(1);
  fresh.timestamp_ns = 1000;
  ASSERT_TRUE(k230_can_batch_is_fresh(fresh, 1050, 100)) << "신선한 묶음";
  ASSERT_FALSE(k230_can_batch_is_fresh(fresh, 1101, 100)) << "낡은 묶음";
  ASSERT_FALSE(k230_can_batch_is_fresh(fresh, 999, 100)) << "미래 시각의 묶음";

  consumer.close();
  producer.close();
}

}  // namespace
