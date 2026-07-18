#include "json_utils.h"

#include <cerrno>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <iomanip>
#include <stdexcept>

// JSON 문자열 값을 출력 스트림에 안전하게 추가한다.
void append_json_string(std::ostringstream *out, const std::string &value) {
  *out << '"';
  for (const char ch : value) {
    switch (ch) {
      case '\\': *out << "\\\\"; break;
      case '"': *out << "\\\""; break;
      case '\n': *out << "\\n"; break;
      case '\r': *out << "\\r"; break;
      case '\t': *out << "\\t"; break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20) {
          *out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
               << static_cast<int>(static_cast<unsigned char>(ch))
               << std::dec << std::setfill(' ');
        } else {
          *out << ch;
        }
        break;
    }
  }
  *out << '"';
}

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
