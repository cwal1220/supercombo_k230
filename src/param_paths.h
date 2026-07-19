#pragma once

#include <cstdlib>
#include <string>

inline std::string k230_params_dir() {
  const char *value = std::getenv("K230_PARAMS_DIR");
  return value && value[0] != '\0' ? std::string(value) : std::string("params");
}

inline std::string k230_param_path(const char *name) {
  const std::string dir = k230_params_dir();
  return dir + "/" + (name ? name : "");
}
