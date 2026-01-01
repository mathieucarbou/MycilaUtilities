// SPDX-License-Identifier: MIT
/*
 * Copyright (C) Mathieu Carbou
 */
#pragma once

#include <string.h>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>

namespace Mycila {
  class string {
    public:
      static inline std::string toLowerCase(std::string str) {
        std::transform(str.begin(), str.end(), str.begin(), ::tolower);
        return str;
      }

      static inline std::string toUpperCase(std::string str) {
        std::transform(str.begin(), str.end(), str.begin(), ::toupper);
        return str;
      }

      static inline std::string trim(std::string s) { // NOLINT
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) { return !std::isspace(ch); }));
        return s;
      }

      static inline bool endsWith(const std::string& str, const std::string_view suffix) {
        return str.length() >= suffix.length() && str.compare(str.length() - suffix.length(), suffix.length(), suffix) == 0;
      }

      static inline bool startsWith(const std::string& str, const char* suffix) {
        return str.rfind(suffix, 0) == 0;
      }

      static inline std::string to_string(float number, int decimals) {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(decimals) << number;
        return stream.str();
      }

      static inline std::string to_string(double number, int decimals) {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(decimals) << number;
        return stream.str();
      }
  };
} // namespace Mycila
