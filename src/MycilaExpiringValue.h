// SPDX-License-Identifier: MIT
/*
 * Copyright (C) Mathieu Carbou
 */
#pragma once

#ifdef MYCILA_JSON_SUPPORT
  #include <ArduinoJson.h>
#endif

#include <optional>
#include <stdexcept>
#include <utility>

namespace Mycila {
  template <typename T>
  class ExpiringValue {
    public:
      explicit ExpiringValue(uint32_t expiration = 0) : _time(0), _expiration(expiration) {}

      // update the value and returns the old value
      std::optional<T> update(T newVale) {
        if (_time) {
          T old = std::forward<T>(_val);
          _val = std::forward<T>(newVale);
          _time = millis();
          return old;
        }
        _val = std::forward<T>(newVale);
        _time = millis();
        return std::nullopt;
      }

      void setExpiration(uint32_t millis) { _expiration = millis; }
      uint32_t getExpiration() const { return _expiration; }

      // Get the last update time
      uint32_t getLastUpdateTime() const { return _time; }
      // Get the elapsed time since the last update
      uint32_t getElapsedTime() const { return millis() - _time; }
      // Check if the value has expired
      bool isExpired() const { return _expiration > 0 && (getElapsedTime() >= _expiration); }
      // Check if we have a valid non-expired value
      bool isPresent() const { return _time > 0 && !isExpired(); }
      bool isAbsent() const { return !isPresent(); }
      // check if the value was never updated (optional is empty since creation)
      bool neverUpdated() const { return _time == 0; }

      // Get the value or an alternative one
      T orElse(T&& alt) const& { return isPresent() ? _val : alt; }
      // Get the value or an alternative one
      const T& orElse(const T& alt) const { return isPresent() ? _val : alt; }

      // Get the optional value
      std::optional<T> opt() const { return isPresent() ? std::optional<T>(_val) : std::nullopt; }

      // get the value (even if expired), throw if never updated
      const T& get() const& {
        if (neverUpdated())
          throw std::runtime_error("No value present");
        return _val;
      }

      // get the value (even if expired), throw if never updated
      T& get() & {
        if (neverUpdated())
          throw std::runtime_error("No value present");
        return _val;
      }

      // reset as if it was never updated
      void reset() { _time = 0; }

      explicit operator bool() const noexcept { return isPresent(); }

#ifdef MYCILA_JSON_SUPPORT
      void toJson(const JsonObject& root) const {
        if (isPresent()) {
          root["expired"] = isExpired();
          root["time"] = _time;
          root["value"] = _val;
        }
      }
#endif

    private:
      uint32_t _time;
      uint32_t _expiration;
      T _val;
  };

} // namespace Mycila
