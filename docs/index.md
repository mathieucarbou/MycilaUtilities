# MycilaUtilities

[![Latest Release](https://img.shields.io/github/release/mathieucarbou/MycilaUtilities.svg)](https://GitHub.com/mathieucarbou/MycilaUtilities/releases/)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/mathieucarbou/library/MycilaUtilities.svg)](https://registry.platformio.org/libraries/mathieucarbou/MycilaUtilities)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Contributor Covenant](https://img.shields.io/badge/Contributor%20Covenant-2.1-4baaaa.svg)](code_of_conduct.md)

[![Build](https://github.com/mathieucarbou/MycilaUtilities/actions/workflows/ci.yml/badge.svg)](https://github.com/mathieucarbou/MycilaUtilities/actions/workflows/ci.yml)
[![GitHub latest commit](https://badgen.net/github/last-commit/mathieucarbou/MycilaUtilities)](https://GitHub.com/mathieucarbou/MycilaUtilities/commit/)
[![Gitpod Ready-to-Code](https://img.shields.io/badge/Gitpod-Ready--to--Code-blue?logo=gitpod)](https://gitpod.io/#https://github.com/mathieucarbou/MycilaUtilities)

Utility helpers for Arduino / ESP32: PID controller, timing helpers, circular buffer, etc.

| [![](https://mathieu.carbou.me/MycilaUtilities/assets/pid-graph.jpeg)](https://mathieu.carbou.me/MycilaUtilities/assets/pid-graph.jpeg) | [![](https://mathieu.carbou.me/MycilaUtilities/assets/pid-console.jpeg)](https://mathieu.carbou.me/MycilaUtilities/assets/pid-console.jpeg) |

## Quick links

- Source: https://github.com/mathieucarbou/MycilaUtilities
- Examples: `examples/`
- API headers: `src/`
- Classes reference: `docs/CLASSES.md`

## Classes and utilities

This repository includes a few focused utility classes. See `docs/CLASSES.md` for the full short reference; the main types are:

- `Mycila::PID` — flexible PID controller with modes, anti-windup, json export (optional), pause/resume and reset.
- `Mycila::CircularBuffer<T,N>` — small fixed-size numeric circular buffer with rolling stats (avg/sum/min/max).
- `Mycila::ExpiringValue<T>` — value container with expiration semantics and optional JSON export.
- `Mycila::string` — string helper utilities (trim, case conversion, formatting floats).
- `Mycila::Time` — helpers for formatting Unix timestamps, converting HH:MM, and time-range checks.

### Mycila::PID (src/MycilaPID.h)

A compact and flexible PID controller. Key features:

- Configurable Kp, Ki, Kd and tunings
- Proportional and derivative modes (ON_ERROR or ON_INPUT)
- Integral correction modes (OFF or CLAMP)
- Output limits and optional anti-windup
- Reverse mode, time-sampling mode, pause/resume, and reset with optional initial output
- State getters (pTerm/iTerm/dTerm/last input/output) and optional JSON export when `MYCILA_JSON_SUPPORT` is enabled

**PID Simulator** available in `examples/PIDSimulator` ([video link](https://youtu.be/aSKE0_tJjhw))

**Full PID controller documentation** is available at [https://mathieu.carbou.me/MycilaUtilities/pid](https://mathieu.carbou.me/MycilaUtilities/pid)

### Mycila::CircularBuffer<T,N> (src/MycilaCircularBuffer.h)

A fixed-size circular buffer specialized for numeric types (integers and floating point).

- O(1) add and rolling statistics: count, sum, avg, min, max, first/last, diff, rate
- Useful for smoothing, moving averages, and small-window metrics

### Mycila::ExpiringValue<T> (src/MycilaExpiringValue.h)

Holds a value with an optional expiration time (milliseconds).

- Update the value and obtain the old value
- Check presence/expiration and retrieve the value or an alternative
- Optional JSON export when `MYCILA_JSON_SUPPORT` is enabled

### Mycila::string (src/MycilaString.h)

Small string helper utilities:

- toLowerCase / toUpperCase
- trim, startsWith, endsWith
- to_string(float,double,decimals)

### Mycila::Time (src/MycilaTime.h)

Time helpers:

- Format Unix timestamps to ISO8601 or local strings
- Convert HH:MM to minutes and check if a time is in a range
- Format seconds into D HH:MM:SS

If you'd like, I can expand this page with example snippets, code references, or copy parts of header comments into the docs.

## Examples

See `examples/` for demonstrations and working sketches, including a PID simulator and web UI examples.

## Contributing

Contributions are welcome — please follow the `CODE_OF_CONDUCT.md` and open PRs against `main`.

## License

MIT
