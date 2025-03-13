// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2025 Mathieu Carbou
 */
#pragma once

#include <inttypes.h>
#include <string.h>
#include <sys/time.h>

#include <cstdio>
#include <string>

namespace Mycila {
  class Time {
    public:
      static std::string toISO8601Str(time_t unixTime) {
        if (unixTime == 0)
          return std::string();
        char buffer[21];
        strftime(buffer, sizeof(buffer), "%FT%TZ", gmtime(&unixTime));
        return buffer;
      }

      static std::string toLocalStr(time_t unixTime) {
        if (unixTime == 0)
          return std::string();
        struct tm timeInfo;
        localtime_r(&unixTime, &timeInfo);
        char buffer[20];
        strftime(buffer, sizeof(buffer), "%F %T", &timeInfo);
        return buffer;
      }

      static time_t getUnixTime() {
        time_t now;
        struct tm timeinfo;
        if (!getLocalTime(&timeinfo, 5))
          return 0;
        time(&now);
        return now;
      }

      static std::string getISO8601Str() { return toISO8601Str(getUnixTime()); }

      static std::string getLocalStr() { return toLocalStr(getUnixTime()); }

      static uint32_t toMinutes(const char* time, char sep = ':') {
        if (!time || !time[0])
          return 0;
        int i = -1;
        for (int j = 0; time[j] != '\0'; j++) {
          if (time[j] == sep) {
            i = j;
            break;
          }
        }
        if (i < 0)
          return atol(time);
        char* hours = reinterpret_cast<char*>(malloc(i + 1));
        memcpy(hours, time, i);
        hours[i] = '\0';
        uint32_t l = atol(hours) * 60 + atol(time + i + 1);
        free(hours);
        return l;
      }

      static uint32_t timeInRange(const struct tm& timeInfo, const char* start, const char* end, char sep = ':') {
        const uint32_t startMinutes = toMinutes(start, sep);
        const uint32_t stopMinutes = toMinutes(end, sep);

        if (startMinutes == stopMinutes)
          return false; // range is empty

        const uint32_t timeMinutes = timeInfo.tm_hour * 60 + timeInfo.tm_min;

        // cases:
        // startMinutes < stopMinutes : i.e. 06:00 -> 22:00
        // startMinutes > stopMinutes  : i.e. 22:00 -> 06:00
        return (startMinutes < stopMinutes && timeMinutes >= startMinutes && timeMinutes < stopMinutes) || (startMinutes > stopMinutes && (timeMinutes >= startMinutes || timeMinutes < stopMinutes));
      }

      static std::string toDHHMMSS(uint32_t seconds) {
        const uint8_t days = seconds / 86400;
        seconds = seconds % static_cast<uint32_t>(86400);
        const uint8_t hh = seconds / 3600;
        seconds = seconds % static_cast<uint32_t>(3600);
        const uint8_t mm = seconds / 60;
        const uint8_t ss = seconds % static_cast<uint32_t>(60);
        char buffer[14];
        snprintf(buffer, sizeof(buffer), "%" PRIu8 "d %02" PRIu8 ":%02" PRIu8 ":%02" PRIu8, days % 1000, hh % 100, mm % 100, ss % 100);
        return buffer;
      }
  };

} // namespace Mycila
