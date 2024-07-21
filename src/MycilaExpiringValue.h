// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2024 Mathieu Carbou and others
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
      std::optional<T> update(const T& newVale) {
        if (_time) {
          T old = _val;
          _val = newVale;
          _time = millis();
          return old;
        }
        _val = newVale;
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
      // Check if we have a valid value
      bool isPresent() const { return _time > 0 && !isExpired(); }
      bool isAbsent() const { return !isPresent(); }
      bool neverUpdated() const { return _time == 0; }

      // Get the value or an alternative one
      T orElse(T&& alt) const& { return isPresent() ? _val : alt; }
      T orElse(T&& alt) && { return isPresent() ? std::move(_val) : alt; }
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

      // reset as if it ws never updated
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
