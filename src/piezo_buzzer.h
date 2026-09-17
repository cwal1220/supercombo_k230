#ifndef PIEZO_BUZZER_H
#define PIEZO_BUZZER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 보드 알림 시퀀스와 engage 음을 포함한다. */
typedef enum PiezoAlert {
  PIEZO_ALERT_SIGNAL_CHANGED = 0,
  PIEZO_ALERT_UNAVAILABLE = 1,
  PIEZO_ALERT_ENGAGE = 2,
  PIEZO_ALERT_DISENGAGE = 3,
  /* openpilot의 거부/"engage 불가" 알림음. */
  PIEZO_ALERT_UNABLE = 4,
  PIEZO_ALERT_COUNT = 5,
} PiezoAlert;

typedef struct PiezoBuzzer PiezoBuzzer;

/* 객체가 워커 스레드를 소유하며 overlay 프레임 루프를 블로킹하지 않는다.
 * enabled=0이면 재생 요청을 조용히 버린다. */
PiezoBuzzer *piezo_buzzer_create(int enabled);
void piezo_buzzer_destroy(PiezoBuzzer *buzzer);
void piezo_buzzer_play(PiezoBuzzer *buzzer, PiezoAlert alert, uint32_t event_id);

#ifdef __cplusplus
}
#endif

#endif  // PIEZO_BUZZER_H
