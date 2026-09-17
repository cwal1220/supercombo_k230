#ifndef UTILS_PROCESS_H
#define UTILS_PROCESS_H

/* 프로세스가 OS에서 받는 것: 환경변수, params 디렉터리 경로, 종료 시그널.
 * 불리언 환경변수 규약은 env_flag 하나다. */

#include <cctype>
#include <csignal>
#include <cstdlib>
#include <string>

/* 환경변수 불리언 공통 규약. 미설정이거나 빈 값이면 default_value,
 * "0"/"false"/"no"/"off"/"n"(대소문자 무시)이면 false, 나머지는 true. */
inline bool env_flag(const char *name, bool default_value = false)
{
    const char *value = std::getenv(name);
    if (!value || value[0] == '\0') return default_value;
    std::string lowered(value);
    for (char &character : lowered)
        character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
    return !(lowered == "0" || lowered == "false" || lowered == "no" ||
             lowered == "off" || lowered == "n");
}

inline bool env_present(const char *name) {
  return std::getenv(name) != nullptr;
}

inline unsigned env_unsigned(const char *name, unsigned default_value) {
  const char *value = std::getenv(name);
  if (!value || value[0] == '\0') return default_value;
  char *end = nullptr;
  const unsigned long parsed = std::strtoul(value, &end, 10);
  return end == value ? default_value : static_cast<unsigned>(parsed);
}

inline float env_float(const char *name, float default_value) {
  const char *value = std::getenv(name);
  if (!value || value[0] == '\0') return default_value;
  char *end = nullptr;
  const float parsed = std::strtof(value, &end);
  return end == value ? default_value : parsed;
}


inline std::string env_string(const char *name, const char *fallback = "") {
  const char *value = std::getenv(name);
  return value && value[0] ? value : fallback;
}

// params 디렉토리 경로 결정 (K230_PARAMS_DIR 재정의 가능)
inline std::string k230_params_dir() {
  const char *value = std::getenv("K230_PARAMS_DIR");
  return value && value[0] != '\0' ? std::string(value) : std::string("params");
}

inline std::string k230_param_path(const char *name) {
  return k230_params_dir() + "/" + (name ? name : "");
}

namespace utils_process_detail {

inline volatile sig_atomic_t *&stop_flag()
{
    static volatile sig_atomic_t *flag = nullptr;
    return flag;
}

inline void stop_signal_handler(int)
{
    if (stop_flag() != nullptr) *stop_flag() = 1;
}

}

inline void install_stop_signal_handlers(volatile sig_atomic_t *stop_flag)
{
    utils_process_detail::stop_flag() = stop_flag;
    signal(SIGINT, utils_process_detail::stop_signal_handler);
    signal(SIGTERM, utils_process_detail::stop_signal_handler);
}

#endif
