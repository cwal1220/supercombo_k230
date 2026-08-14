#include "json_utils.h"

#include "common_utils.h"

#include <cerrno>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <stdexcept>

// 단순 JSON 텍스트에서 bool 값을 읽는다.
bool parse_json_bool_value(const std::string &text, const std::string &key, bool *out) {
  if (!out) return false;
  const std::string quoted_key = "\"" + key + "\"";
  const size_t key_pos = text.find(quoted_key);
  if (key_pos == std::string::npos) return false;

  const size_t colon = text.find(':', key_pos + quoted_key.size());
  if (colon == std::string::npos) {
    throw std::runtime_error("json key '" + key + "' has no value");
  }
  size_t cursor = colon + 1;
  while (cursor < text.size() && std::isspace(static_cast<unsigned char>(text[cursor]))) ++cursor;
  if (text.compare(cursor, 4, "true") == 0) {
    *out = true;
    return true;
  }
  if (text.compare(cursor, 5, "false") == 0) {
    *out = false;
    return true;
  }
  throw std::runtime_error("json key '" + key + "' is not a boolean");
}

// 단순 JSON 텍스트에서 float 값을 읽는다.
bool parse_json_float_value(const std::string &text, const std::string &key, float *out) {
  if (!out) return false;
  const std::string quoted_key = "\"" + key + "\"";
  const size_t key_pos = text.find(quoted_key);
  if (key_pos == std::string::npos) return false;

  const size_t colon = text.find(':', key_pos + quoted_key.size());
  if (colon == std::string::npos) {
    throw std::runtime_error("json key '" + key + "' has no value");
  }

  const char *cursor = text.c_str() + colon + 1;
  while (*cursor != '\0' && std::isspace(static_cast<unsigned char>(*cursor))) ++cursor;

  char *end = nullptr;
  errno = 0;
  const float parsed = std::strtof(cursor, &end);
  if (end == cursor || errno == ERANGE || !std::isfinite(parsed)) {
    throw std::runtime_error("json key '" + key + "' contains an invalid float");
  }
  *out = parsed;
  return true;
}

void parse_json_optional_bool(const std::string &text, const std::string &key,
                              bool *field) {
  bool value = false;
  if (parse_json_bool_value(text, key, &value)) *field = value;
}

void parse_json_optional_float(const std::string &text, const std::string &key,
                               float minimum, float maximum, float *field) {
  float value = 0.0f;
  if (parse_json_float_value(text, key, &value))
    *field = clamp_float(value, minimum, maximum);
}

void parse_json_optional_int(const std::string &text, const std::string &key,
                             int minimum, int maximum, int *field) {
  float value = 0.0f;
  if (parse_json_float_value(text, key, &value))
    *field = clamp_int(value, minimum, maximum);
}
