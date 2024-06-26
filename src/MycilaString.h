// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2024 Mathieu Carbou and others
 */
#pragma once

#include <WString.h>

namespace Mycila {
  class Str {
    public:
      static ::String& lowerCase(::String& s) { // NOLINT (runtime/references)
        s.toLowerCase();
        return s;
      }

      static ::String lowerCaseCopy(const ::String& s) {
        ::String copy = s;
        copy.toLowerCase();
        return copy;
      }

      static ::String& upperCase(::String& s) { // NOLINT (runtime/references)
        s.toUpperCase();
        return s;
      }

      static ::String upperCaseCopy(const ::String& s) {
        ::String copy = s;
        copy.toUpperCase();
        return copy;
      }
  };
} // namespace Mycila
