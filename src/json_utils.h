#pragma once

#include <array>
#include <cerrno>
#include <cmath>
#include <cctype>
#include <cstddef>
#include <cstdlib>
#include <stdexcept>
#include <string>

// 단순 JSON 텍스트에서 bool 값을 읽는다.
bool parse_json_bool_value(const std::string &text, const std::string &key, bool *out);

// 단순 JSON 텍스트에서 float 값을 읽는다.
bool parse_json_float_value(const std::string &text, const std::string &key, float *out);

// 단순 JSON 텍스트에서 고정 길이 float 배열을 읽는다.
template <size_t N>
bool parse_json_float_array(const std::string &text, const std::string &key,
                            std::array<float, N> *out) {
  if (!out) return false;
  const std::string quoted_key = "\"" + key + "\"";
  const size_t key_pos = text.find(quoted_key);
  if (key_pos == std::string::npos) return false;

  const size_t open = text.find('[', key_pos + quoted_key.size());
  if (open == std::string::npos) {
    throw std::runtime_error("json key '" + key + "' is not an array");
  }
  const size_t close = text.find(']', open + 1);
  if (close == std::string::npos) {
    throw std::runtime_error("json key '" + key + "' has no closing bracket");
  }

  const char *cursor = text.c_str() + open + 1;
  const char *array_end = text.c_str() + close;
  for (size_t i = 0; i < out->size(); ++i) {
    while (cursor < array_end &&
           (std::isspace(static_cast<unsigned char>(*cursor)) || *cursor == ',')) {
      ++cursor;
    }
    if (cursor >= array_end) {
      throw std::runtime_error("json key '" + key + "' must contain " +
                               std::to_string(out->size()) + " floats");
    }
    char *end = nullptr;
    errno = 0;
    const float parsed = std::strtof(cursor, &end);
    if (end == cursor || errno == ERANGE || !std::isfinite(parsed)) {
      throw std::runtime_error("json key '" + key + "' contains an invalid float");
    }
    (*out)[i] = parsed;
    cursor = end;
  }

  while (cursor < array_end) {
    if (!(std::isspace(static_cast<unsigned char>(*cursor)) || *cursor == ',')) {
      throw std::runtime_error("json key '" + key + "' must contain exactly " +
                               std::to_string(out->size()) + " floats");
    }
    ++cursor;
  }
  return true;
}
