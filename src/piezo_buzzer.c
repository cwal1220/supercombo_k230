#define _POSIX_C_SOURCE 200809L

#include "piezo_buzzer.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define PWM_ROOT "/sys/class/pwm"
#define IOMUX_BASE ((off_t)0x91105000)
#define IOMUX_RW_BITS_MASK 0x00003FFFu
#define IOMUX_IO_SEL_SHIFT 11u
#define IOMUX_IO_SEL_MASK 0x7u
#define IOMUX_IE_SHIFT 8u
#define IOMUX_OE_SHIFT 7u
#define PIEZO_DEFAULT_PIN 46
#define PIEZO_DEFAULT_DUTY 50u

typedef struct {
  unsigned frequency_hz;
  unsigned duration_ms;
  unsigned duty_percent;
} PiezoTone;

typedef struct {
  int pin;
  int pwm_chip;
  int pwm_channel;
  unsigned iomux_alt;
} PiezoPin;

typedef struct {
  PiezoPin pin;
  char pwm_path[96];
  char export_path[96];
  char unexport_path[96];
  int export_requested;
  int exported;
  int enabled;
  int iomux_fd;
  uint32_t iomux_original;
  int iomux_selected;
} PiezoPwm;

struct PiezoBuzzer {
  pthread_mutex_t mutex;
  pthread_cond_t condition;
  pthread_t thread;
  int thread_started;
  int enabled;
  int stop;
  int pending;
  PiezoAlert pending_alert;
  uint32_t pending_event_id;
  int error_reported;
};

/* 모든 알림음은 보드에서 검증한 고정 duty 시퀀스를 사용한다. PCM envelope를
 * 흉내 내려고 25 ms마다 duty를 바꾸면 sysfs 갱신 때 PWM이 잠시 꺼져 passive
 * piezo에서 클릭음이 발생한다. */
static const PiezoTone kSignalChanged[] = {
    {392, 65, PIEZO_DEFAULT_DUTY}, {0, 15, 0},
    {523, 65, PIEZO_DEFAULT_DUTY}, {0, 15, 0},
    {659, 65, PIEZO_DEFAULT_DUTY}, {0, 15, 0},
    {784, 180, PIEZO_DEFAULT_DUTY},
};
static const PiezoTone kActivated[] = {
    {523, 50, PIEZO_DEFAULT_DUTY}, {0, 10, 0},
    {659, 50, PIEZO_DEFAULT_DUTY}, {0, 10, 0},
    {784, 50, PIEZO_DEFAULT_DUTY}, {0, 10, 0},
    {1047, 130, PIEZO_DEFAULT_DUTY}, {0, 15, 0},
    {784, 65, PIEZO_DEFAULT_DUTY}, {1047, 200, PIEZO_DEFAULT_DUTY},
};
static const PiezoTone kDeactivated[] = {
    {784, 70, PIEZO_DEFAULT_DUTY}, {0, 15, 0},
    {659, 70, PIEZO_DEFAULT_DUTY}, {0, 15, 0},
    {523, 70, PIEZO_DEFAULT_DUTY}, {0, 15, 0},
    {440, 80, PIEZO_DEFAULT_DUTY}, {0, 15, 0},
    {392, 190, PIEZO_DEFAULT_DUTY},
};
static const PiezoTone kUnavailable[] = {
    {330, 80, PIEZO_DEFAULT_DUTY}, {0, 25, 0},
    {262, 80, PIEZO_DEFAULT_DUTY}, {0, 25, 0},
    {330, 80, PIEZO_DEFAULT_DUTY}, {0, 25, 0},
    {262, 80, PIEZO_DEFAULT_DUTY}, {0, 60, 0},
    {220, 210, PIEZO_DEFAULT_DUTY},
};
/* openpilot refuse.wav를 근사한 고음-저음-고음 시퀀스. */
static const PiezoTone kUnable[] = {
    {740, 160, PIEZO_DEFAULT_DUTY}, {0, 30, 0},
    {523, 200, PIEZO_DEFAULT_DUTY}, {0, 30, 0},
    {740, 400, PIEZO_DEFAULT_DUTY},
};

static const PiezoTone *const kSequences[PIEZO_ALERT_COUNT] = {
    kSignalChanged, kActivated, kDeactivated, kUnavailable,
    kActivated, kDeactivated, kUnable,
};
static const size_t kSequenceLengths[PIEZO_ALERT_COUNT] = {
    sizeof(kSignalChanged) / sizeof(kSignalChanged[0]),
    sizeof(kActivated) / sizeof(kActivated[0]),
    sizeof(kDeactivated) / sizeof(kDeactivated[0]),
    sizeof(kUnavailable) / sizeof(kUnavailable[0]),
    sizeof(kActivated) / sizeof(kActivated[0]),
    sizeof(kDeactivated) / sizeof(kDeactivated[0]),
    sizeof(kUnable) / sizeof(kUnable[0]),
};
static const char *const kAlertNames[PIEZO_ALERT_COUNT] = {
    "signal_changed", "activated", "deactivated", "unavailable",
    "engage", "disengage", "unable",
};

static int env_is_disabled(const char *name)
{
  const char *value = getenv(name);
  return value && (!strcmp(value, "0") || !strcmp(value, "false") ||
                   !strcmp(value, "FALSE") || !strcmp(value, "off") ||
                   !strcmp(value, "OFF"));
}

static int parse_pin(void)
{
  const char *value = getenv("K230_PIEZO_PIN");
  if (!value || !*value) return PIEZO_DEFAULT_PIN;
  const int pin = atoi(value);
  return pin == 46 || pin == 47 ? pin : PIEZO_DEFAULT_PIN;
}

static PiezoPin pin_config(int pin)
{
  /* 보드의 피에조 핀과 PWM 설정에 맞춘다. */
  if (pin == 47) return (PiezoPin){47, 3, 0, 2};
  return (PiezoPin){46, 0, 2, 2};
}

static int write_sysfs(const char *path, unsigned value)
{
  char text[32];
  const int length = snprintf(text, sizeof(text), "%u", value);
  if (length <= 0 || (size_t)length >= sizeof(text)) return -1;

  const int fd = open(path, O_WRONLY | O_CLOEXEC);
  if (fd < 0) return -1;
  const ssize_t written = write(fd, text, (size_t)length);
  const int saved_errno = errno;
  close(fd);
  if (written != length) {
    errno = saved_errno;
    return -1;
  }
  return 0;
}

static const char *const kDevmemPaths[] = {
    "/sbin/devmem", "/usr/sbin/devmem", "/bin/devmem", "/usr/bin/devmem",
};

/*
 * K230의 /dev/mem 매핑은 pread/pwrite로 seek할 수 없다(이 커널에서는 pread가
 * EFAULT를 반환한다). 따라서 보드 Python 도우미와 동일하게 레지스터 읽기는
 * 매핑으로 처리하고 쓰기는 devmem을 사용한다. IOMUX에 반영되지 않는 쓰기를
 * 조용히 성공 처리하지 않도록 같은 접근 방식을 유지한다.
 */
static void exec_devmem(const char *address, const char *value,
                        int output_fd)
{
  if (output_fd >= 0) {
    if (dup2(output_fd, STDOUT_FILENO) < 0) _exit(126);
    close(output_fd);
  }
  for (size_t i = 0; i < sizeof(kDevmemPaths) / sizeof(kDevmemPaths[0]); ++i) {
    if (value) {
      execl(kDevmemPaths[i], "devmem", address, "32", value,
            (char *)NULL);
    } else {
      execl(kDevmemPaths[i], "devmem", address, "32", (char *)NULL);
    }
  }
  _exit(127);
}

static int wait_for_child(pid_t child)
{
  int status = 0;
  while (waitpid(child, &status, 0) < 0) {
    if (errno != EINTR) return -1;
  }
  return WIFEXITED(status) && WEXITSTATUS(status) == 0 ? 0 : -1;
}

static void iomux_address(int pin, char *text, size_t text_size)
{
  const unsigned long long address =
      (unsigned long long)(IOMUX_BASE + (off_t)pin * 4);
  (void)snprintf(text, text_size, "0x%llx", address);
}

static int devmem_write_iomux(int pin, uint32_t value)
{
  char address[32];
  char value_text[32];
  iomux_address(pin, address, sizeof(address));
  (void)snprintf(value_text, sizeof(value_text), "0x%08x", value);

  const pid_t child = fork();
  if (child < 0) return -1;
  if (child == 0) exec_devmem(address, value_text, -1);
  return wait_for_child(child);
}

static int devmem_read_iomux(int pin, uint32_t *value)
{
  if (!value) return -1;
  char address[32];
  iomux_address(pin, address, sizeof(address));

  int pipe_fds[2];
  if (pipe(pipe_fds) != 0) return -1;
  const pid_t child = fork();
  if (child < 0) {
    close(pipe_fds[0]);
    close(pipe_fds[1]);
    return -1;
  }
  if (child == 0) {
    close(pipe_fds[0]);
    exec_devmem(address, NULL, pipe_fds[1]);
  }

  close(pipe_fds[1]);
  char output[64];
  size_t used = 0;
  while (used + 1 < sizeof(output)) {
    const ssize_t count = read(pipe_fds[0], output + used,
                               sizeof(output) - used - 1);
    if (count > 0) {
      used += (size_t)count;
      continue;
    }
    if (count < 0 && errno == EINTR) continue;
    break;
  }
  close(pipe_fds[0]);
  if (wait_for_child(child) != 0 || used == 0) return -1;

  output[used] = '\0';
  char *end = NULL;
  errno = 0;
  const unsigned long parsed = strtoul(output, &end, 0);
  if (end == output || errno == ERANGE || parsed > UINT32_MAX) return -1;
  *value = (uint32_t)parsed;
  return 0;
}

static int read_iomux(int fd, int pin, uint32_t *value)
{
  if (!value) return -1;
  const long page_size = sysconf(_SC_PAGESIZE);
  if (page_size <= 0) return -1;
  const off_t address = IOMUX_BASE + (off_t)pin * 4;
  const off_t base = address & ~((off_t)page_size - 1);
  const size_t offset = (size_t)(address - base);
  const size_t map_size = (size_t)page_size;
  void *mapped = mmap(NULL, map_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                      base);
  if (mapped == MAP_FAILED) return -1;
  memcpy(value, (const char *)mapped + offset, sizeof(*value));
  (void)munmap(mapped, map_size);
  return 0;
}

static int write_iomux(int pin, uint32_t value)
{
  if (devmem_write_iomux(pin, value) != 0) return -1;

  /* 보드 Python 구현과 동일한 devmem 경로로 쓰기 결과를 검증한다. */
  uint32_t verified = 0;
  if (devmem_read_iomux(pin, &verified) != 0) return -1;
  return (verified & IOMUX_RW_BITS_MASK) == (value & IOMUX_RW_BITS_MASK) ? 0
                                                                          : -1;
}

static int iomux_select(PiezoPwm *pwm)
{
  const int fd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC);
  if (fd < 0) return -1;

  uint32_t original = 0;
  int result = read_iomux(fd, pwm->pin.pin, &original);
  if (result == 0) {
    uint32_t value = original;
    value &= ~(IOMUX_IO_SEL_MASK << IOMUX_IO_SEL_SHIFT);
    value |= pwm->pin.iomux_alt << IOMUX_IO_SEL_SHIFT;
    value &= ~(1u << IOMUX_IE_SHIFT);
    value |= 1u << IOMUX_OE_SHIFT;
    result = write_iomux(pwm->pin.pin, value);
    if (result == 0) {
      pwm->iomux_fd = fd;
      pwm->iomux_original = original;
      pwm->iomux_selected = 1;
      return 0;
    }
  }
  close(fd);
  return result;
}

static void iomux_restore(PiezoPwm *pwm)
{
  if (!pwm->iomux_selected) return;
  (void)write_iomux(pwm->pin.pin, pwm->iomux_original);
  close(pwm->iomux_fd);
  pwm->iomux_fd = -1;
  pwm->iomux_selected = 0;
}

static void pwm_paths(PiezoPwm *pwm)
{
  snprintf(pwm->pwm_path, sizeof(pwm->pwm_path), "%s/pwmchip%d/pwm%d",
           PWM_ROOT, pwm->pin.pwm_chip, pwm->pin.pwm_channel);
  snprintf(pwm->export_path, sizeof(pwm->export_path), "%s/pwmchip%d/export",
           PWM_ROOT, pwm->pin.pwm_chip);
  snprintf(pwm->unexport_path, sizeof(pwm->unexport_path),
           "%s/pwmchip%d/unexport", PWM_ROOT, pwm->pin.pwm_chip);
}

static void pwm_stop(PiezoPwm *pwm)
{
  if (pwm->exported) {
    char enable_path[128];
    snprintf(enable_path, sizeof(enable_path), "%s/enable", pwm->pwm_path);
    (void)write_sysfs(enable_path, 0);
  }
  iomux_restore(pwm);
  if (pwm->export_requested)
    (void)write_sysfs(pwm->unexport_path, (unsigned)pwm->pin.pwm_channel);
  pwm->export_requested = 0;
  pwm->exported = 0;
  pwm->enabled = 0;
}

static int tone_values(unsigned frequency_hz, unsigned duty_percent,
                       unsigned *period_ns, unsigned *duty_ns)
{
  if (frequency_hz == 0 || duty_percent == 0 || duty_percent >= 100 ||
      !period_ns || !duty_ns) {
    return -1;
  }
  const uint64_t period = (1000000000ULL + frequency_hz / 2) / frequency_hz;
  if (period == 0 || period > UINT32_MAX) return -1;
  *period_ns = (unsigned)period;
  *duty_ns = (unsigned)((period * duty_percent + 50) / 100);
  return 0;
}

static int pwm_start(PiezoPwm *pwm, unsigned frequency_hz,
                     unsigned duty_percent)
{
  unsigned period_ns = 0;
  unsigned duty_ns = 0;
  if (tone_values(frequency_hz, duty_percent, &period_ns, &duty_ns) != 0)
    return -1;
  if (access(pwm->pwm_path, F_OK) == 0) return -1;

  if (write_sysfs(pwm->export_path, (unsigned)pwm->pin.pwm_channel) != 0)
    return -1;
  pwm->export_requested = 1;

  for (unsigned attempt = 0; attempt < 20; ++attempt) {
    if (access(pwm->pwm_path, F_OK) == 0) break;
    struct timespec delay = {0, 10000000L};
    nanosleep(&delay, NULL);
  }
  if (access(pwm->pwm_path, F_OK) != 0) {
    pwm_stop(pwm);
    return -1;
  }
  pwm->exported = 1;

  char path[128];
  snprintf(path, sizeof(path), "%s/enable", pwm->pwm_path);
  if (write_sysfs(path, 0) != 0) goto fail;
  snprintf(path, sizeof(path), "%s/period", pwm->pwm_path);
  if (write_sysfs(path, period_ns) != 0) goto fail;
  snprintf(path, sizeof(path), "%s/duty_cycle", pwm->pwm_path);
  if (write_sysfs(path, duty_ns) != 0) goto fail;
  if (iomux_select(pwm) != 0) goto fail;
  snprintf(path, sizeof(path), "%s/enable", pwm->pwm_path);
  if (write_sysfs(path, 1) != 0) goto fail;
  pwm->enabled = 1;
  return 0;

fail:
  pwm_stop(pwm);
  return -1;
}

static int pwm_set_frequency(PiezoPwm *pwm, unsigned frequency_hz,
                             unsigned duty_percent)
{
  unsigned period_ns = 0;
  unsigned duty_ns = 0;
  if (!pwm->exported ||
      tone_values(frequency_hz, duty_percent, &period_ns, &duty_ns) != 0)
    return -1;
  char path[128];
  if (pwm->enabled) {
    snprintf(path, sizeof(path), "%s/enable", pwm->pwm_path);
    if (write_sysfs(path, 0) != 0) return -1;
  }
  snprintf(path, sizeof(path), "%s/period", pwm->pwm_path);
  if (write_sysfs(path, period_ns) != 0) return -1;
  snprintf(path, sizeof(path), "%s/duty_cycle", pwm->pwm_path);
  if (write_sysfs(path, duty_ns) != 0) return -1;
  if (pwm->enabled) {
    snprintf(path, sizeof(path), "%s/enable", pwm->pwm_path);
    if (write_sysfs(path, 1) != 0) return -1;
  }
  return 0;
}

static int pwm_set_enabled(PiezoPwm *pwm, int enabled)
{
  if (!pwm->exported) return -1;
  char path[128];
  snprintf(path, sizeof(path), "%s/enable", pwm->pwm_path);
  if (write_sysfs(path, enabled ? 1u : 0u) != 0) return -1;
  pwm->enabled = enabled != 0;
  return 0;
}

static int play_sequence(PiezoAlert alert)
{
  if (alert < 0 || alert >= PIEZO_ALERT_COUNT) return -1;

  PiezoPwm pwm = {0};
  pwm.pin = pin_config(parse_pin());
  pwm.iomux_fd = -1;
  pwm_paths(&pwm);
  int result = 0;

  for (size_t i = 0; i < kSequenceLengths[alert]; ++i) {
    const PiezoTone tone = kSequences[alert][i];
    if (tone.frequency_hz == 0) {
      if (pwm.enabled && pwm_set_enabled(&pwm, 0) != 0) {
        result = -1;
        break;
      }
    } else if (!pwm.exported) {
      if (pwm_start(&pwm, tone.frequency_hz, tone.duty_percent) != 0) {
        result = -1;
        break;
      }
    } else {
      if (pwm_set_frequency(&pwm, tone.frequency_hz, tone.duty_percent) != 0) {
        result = -1;
        break;
      }
      if (!pwm.enabled && pwm_set_enabled(&pwm, 1) != 0) {
        result = -1;
        break;
      }
    }
    struct timespec delay = {
        (time_t)(tone.duration_ms / 1000),
        (long)(tone.duration_ms % 1000) * 1000000L};
    while (nanosleep(&delay, &delay) != 0 && errno == EINTR) {}
  }
  pwm_stop(&pwm);
  return result;
}

static void report_unavailable(PiezoBuzzer *buzzer, PiezoAlert alert,
                               uint32_t event_id)
{
  pthread_mutex_lock(&buzzer->mutex);
  const int report = !buzzer->error_reported;
  if (report) buzzer->error_reported = 1;
  pthread_mutex_unlock(&buzzer->mutex);
  if (report) {
    fprintf(stderr,
            "k230_overlayd: piezo buzzer unavailable, continuing with LCD "
            "alerts (alert=%s event=%u)\n",
            alert >= 0 && alert < PIEZO_ALERT_COUNT ? kAlertNames[alert] :
                                                       "unknown",
            event_id);
  }
}

static void *piezo_thread(void *opaque)
{
  PiezoBuzzer *buzzer = (PiezoBuzzer *)opaque;
  for (;;) {
    pthread_mutex_lock(&buzzer->mutex);
    while (!buzzer->stop && !buzzer->pending)
      pthread_cond_wait(&buzzer->condition, &buzzer->mutex);
    if (buzzer->stop) {
      pthread_mutex_unlock(&buzzer->mutex);
      return NULL;
    }
    const PiezoAlert alert = buzzer->pending_alert;
    const uint32_t event_id = buzzer->pending_event_id;
    buzzer->pending = 0;
    pthread_mutex_unlock(&buzzer->mutex);

    if (play_sequence(alert) != 0) report_unavailable(buzzer, alert, event_id);
  }
}

PiezoBuzzer *piezo_buzzer_create(void)
{
  PiezoBuzzer *buzzer = (PiezoBuzzer *)calloc(1, sizeof(*buzzer));
  if (!buzzer) return NULL;
  if (pthread_mutex_init(&buzzer->mutex, NULL) != 0) {
    free(buzzer);
    return NULL;
  }
  if (pthread_cond_init(&buzzer->condition, NULL) != 0) {
    pthread_mutex_destroy(&buzzer->mutex);
    free(buzzer);
    return NULL;
  }
  buzzer->enabled = !env_is_disabled("K230_PIEZO_BUZZER");
  if (buzzer->enabled && pthread_create(&buzzer->thread, NULL, piezo_thread,
                                        buzzer) != 0) {
    pthread_cond_destroy(&buzzer->condition);
    pthread_mutex_destroy(&buzzer->mutex);
    free(buzzer);
    return NULL;
  }
  buzzer->thread_started = buzzer->enabled;
  return buzzer;
}

void piezo_buzzer_destroy(PiezoBuzzer *buzzer)
{
  if (!buzzer) return;
  if (buzzer->thread_started) {
    pthread_mutex_lock(&buzzer->mutex);
    buzzer->stop = 1;
    pthread_cond_signal(&buzzer->condition);
    pthread_mutex_unlock(&buzzer->mutex);
    pthread_join(buzzer->thread, NULL);
  }
  pthread_cond_destroy(&buzzer->condition);
  pthread_mutex_destroy(&buzzer->mutex);
  free(buzzer);
}

void piezo_buzzer_play(PiezoBuzzer *buzzer, PiezoAlert alert, uint32_t event_id)
{
  if (!buzzer || !buzzer->enabled || event_id == 0 ||
      alert < 0 || alert >= PIEZO_ALERT_COUNT) {
    return;
  }
  pthread_mutex_lock(&buzzer->mutex);
  if (!buzzer->stop) {
    buzzer->pending_alert = alert;
    buzzer->pending_event_id = event_id;
    buzzer->pending = 1;
    pthread_cond_signal(&buzzer->condition);
  }
  pthread_mutex_unlock(&buzzer->mutex);
}
