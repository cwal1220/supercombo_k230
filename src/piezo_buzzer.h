#ifndef PIEZO_BUZZER_H
#define PIEZO_BUZZER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 보드 알림 시퀀스와 engage 음을 포함한다. */
typedef enum PiezoAlert {
  PIEZO_ALERT_SIGNAL_CHANGED = 0,
  PIEZO_ALERT_ACTIVATED = 1,
  PIEZO_ALERT_DEACTIVATED = 2,
  PIEZO_ALERT_UNAVAILABLE = 3,
  PIEZO_ALERT_ENGAGE = 4,
  PIEZO_ALERT_DISENGAGE = 5,
  /* openpilot의 거부/"engage 불가" 알림음. */
  PIEZO_ALERT_UNABLE = 6,
  PIEZO_ALERT_COUNT = 7,
} PiezoAlert;

typedef struct PiezoBuzzer PiezoBuzzer;

/* 객체가 워커 스레드를 소유하며 overlay 프레임 루프를 블로킹하지 않는다. */
PiezoBuzzer *piezo_buzzer_create(void);
void piezo_buzzer_destroy(PiezoBuzzer *buzzer);
void piezo_buzzer_play(PiezoBuzzer *buzzer, PiezoAlert alert, uint32_t event_id);

#ifdef __cplusplus
}
#endif

#endif  // PIEZO_BUZZER_H
