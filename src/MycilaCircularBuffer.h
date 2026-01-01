// SPDX-License-Identifier: MIT
/*
 * Copyright (C) Mathieu Carbou
 */
#pragma once

#include <Print.h>
#include <stddef.h>

#include <functional>
#include <limits>

namespace Mycila {
  template <typename T, size_t N, std::enable_if_t<std::is_integral_v<T> || std::is_floating_point_v<T>, bool> = true>
  class CircularBuffer {
    public:
      CircularBuffer() { reset(); }

      T const& operator[](size_t index) const { return _buffer[(_index + index) % N]; }

      size_t count() const { return _count; };
      T avg() const { return _count == 0 ? 0 : _sum / _count; }
      T first() const { return _buffer[_index]; }
      T last() const { return _buffer[_index == 0 ? N - 1 : _index - 1]; }
      T sum() const { return _sum; }
      T max() const { return _max; }
      T min() const { return _min; }
      T diff() const { return last() - first(); }
      T rate() const {
        T d = diff();
        return d == 0 ? 0 : _count / d;
      }

      T add(T value) {
        T current = _buffer[_index];
        _buffer[_index++] = value;
        if (value > _max)
          _max = value;
        if (value < _min)
          _min = value;
        _sum += value;
        _sum -= current;
        if (_index == N)
          _index = 0;
        if (_count < N)
          _count++;
        return current;
      };

      void copy(T* dest) const { // NOLINT (build/include_what_you_use)
        const size_t start = _index;
        for (size_t i = 0; i < N; i++)
          dest[i] = _buffer[(start + i) % N];
      }

      void reset() {
        _sum = 0;
        _min = std::numeric_limits<T>::max();
        _max = std::numeric_limits<T>::min();
        _index = 0;
        _count = 0;
        for (int i = 0; i < N; i++)
          _buffer[i] = 0;
      }

      void dump(Print& printer) { // NOLINT (runtime/references)
        printer.print("CircularBuffer(");
        printer.print(_index);
        printer.print(",");
        printer.print(_count);
        printer.print("/");
        printer.print(N);
        printer.print(")={");
        for (size_t i = 0; i < N; i++) {
          printer.print(_buffer[i]);
          if (i < N - 1)
            printer.print(",");
        }
        printer.print("}");
      }

    private:
      T _buffer[N];
      T _sum;
      T _min;
      T _max;
      size_t _index;
      size_t _count;
  };

} // namespace Mycila
