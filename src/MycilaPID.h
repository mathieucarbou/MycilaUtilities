// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2025 Mathieu Carbou
 */
#pragma once

#ifdef MYCILA_JSON_SUPPORT
  #include <ArduinoJson.h>
#endif

#include <Print.h>

namespace Mycila {
  /**
   * @brief A simple and efficient PID controller implementation inspired by:
   * @brief - https://github.com/Dlloydev/QuickPID
   * @brief - https://github.com/br3ttb/Arduino-PID-Library
   * @brief - https://github.com/m-lundberg/simple-pid
   */
  class PID {
    public:
      /**
       * @brief Proportional control modes: determines how the proportional term is calculated.
       */
      enum class ProportionalMode {
        /**
         * @brief Proportional term computed from the error (default and traditional method)
         */
        ON_ERROR,
        /**
         * @brief Proportional term computed from the input
         */
        ON_INPUT,
      };

      /**
       * @brief Integral correction modes: determines how the integral term is clamped to prevent large overshoots (windup protection).
       */
      enum class IntegralCorrectionMode {
        /**
         * @brief No integral correction (not recommended)
         */
        OFF,
        /**
         * @brief Clamp the integral term between the output limits (default and traditional method)
         * @brief This prevents the integral term from accumulating beyond the output limits, helping to reduce overshoot.
         * @brief However, it may lead to slower response times in some scenarios.
         * @brief Requires output limits to be set.
         */
        CLAMP,
      };

    public:
      ProportionalMode getProportionalMode() const { return _pMode; }
      IntegralCorrectionMode getIntegralCorrectionMode() const { return _icMode; }
      bool isReverse() const { return _reverse; }
      bool isEnabled() const { return _enabled; }
      bool isTimeSampling() const { return _timeSampling; }

      float getSetpoint() const { return _setpoint; }
      float getKp() const { return _kp; }
      float getKi() const { return _ki; }
      float getKd() const { return _kd; }
      float getOutputMin() const { return _outputMin; }
      float getOutputMax() const { return _outputMax; }

      float getInput() const { return _lastInput; }
      float getOutput() const { return _lastOutput; }

      float getPTerm() const { return _pTerm; }
      float getITerm() const { return _iTerm; }
      float getDTerm() const { return _dTerm; }
      float getFeedForward() const { return _feed; }

      /**
       * @brief Get the filter coefficient (alpha) for the first-order lag filter.
       * @return The filter coefficient (0 = heavy filtering, 1 = no filtering)
       */
      float getFilterAlpha() const { return _filterAlpha; }

      /**
       * @brief Get the last time (in microseconds) when compute() was called.
       * @return The last time in microseconds, or 0 if compute() has never been called.
       */
      uint32_t getLastTime() const { return _lastTime; }

      void setProportionalMode(ProportionalMode mode) { _pMode = mode; }
      void setIntegralCorrectionMode(IntegralCorrectionMode mode) { _icMode = mode; }

      /**
       * @brief Set the controller direction.
       * @brief In reverse mode, the output will decrease as the input increases.
       * @brief This is useful for applications where the actuator works in reverse, such as cooling systems, which work to decrease temperature as the input increases.
       * @brief At the opposite, in normal mode, the output will increase as the input increases, which is useful for heating systems.
       * @param reverse true to set reverse mode, false for normal mode (default).
       */
      void setReverse(bool reverse) { _reverse = reverse; }

      /**
       * @brief Set whether the PID controller is time-sampling based or not.
       * @brief In time-sampling mode, the PID computation takes into account the time elapsed
       * since the last computation, allowing for variable time intervals between calls to compute().
       * @brief This is useful for applications where the compute() method may be called at irregular
       * intervals, ensuring that the PID calculations remain accurate regardless of the timing of the calls.
       * @brief When time-sampling is disabled, the PID computation assumes a constant time interval
       * between calls to compute(), which may be suitable for applications with a fixed control loop timing.
       * @param timeSampling true to enable time-sampling mode, false to disable it (default).
       */
      void setTimeSampling(bool timeSampling) { _timeSampling = timeSampling; }

      // PID target
      void setSetpoint(float setpoint) { _setpoint = setpoint; }

      // PID proportional gain
      void setKp(float kp) { _kp = kp; }
      // PID integral gain
      void setKi(float ki) { _ki = ki; }
      // PID derivative gain
      void setKd(float kd) { _kd = kd; }

      /**
       * @brief Set the feed-forward term.
       * @brief Feed-forward adds a known expected output value to the PID output, improving response time and reducing steady-state error.
       * @brief Useful when the relationship between setpoint and required output is partially known (e.g., precompensation for known disturbances).
       * @param feedForward The feed-forward value to add to the PID output (default is 0).
       */
      void setFeedForward(float feedForward) { _feed = feedForward; }

      /**
       * @brief Set the filter coefficient (alpha) for first-order lag filtering of the input.
       * @brief This implements a first-order lag filter (exponential smoothing) to reduce measurement noise.
       * @brief Filtered_new = alpha * input + (1 - alpha) * Filtered_old
       * @brief Unlike moving-average filters, this gives more weight to recent samples and exponentially decreases older samples.
       * @param alpha Filter coefficient between 0 and 1 (default 1 = no filtering)
       *              - 0.0 to 0.3: Heavy filtering (slow response, max noise reduction)
       *              - 0.3 to 0.7: Moderate filtering (balanced)
       *              - 0.7 to 1.0: Light filtering (fast response, minimal smoothing)
       *              - 1.0: No filtering (raw input)
       * @note For JSY measurements at 330ms intervals, try alpha = 0.3 to 0.5
       */
      void setFilterAlpha(float alpha) {
        _filterAlpha = constrain(alpha, 0.0f, 1.0f);
      }

      /**
       * @brief Set the filter time constant (tau) in seconds for the first-order lag filter.
       * @brief The filter will reach ~63% of a step change in tau seconds.
       * @brief This calculates alpha = sampleTime / (tau + sampleTime)
       * @param tau Time constant in seconds (e.g., 1.0 = 1 second settling time)
       * @param sampleTime Expected time between compute() calls in seconds (e.g., 0.33 for 330ms)
       * @note Recommended: tau = 0.5 to 2.0 seconds for JSY at 330ms sample rate
       * @note tau = 1.0s gives alpha ≈ 0.25 (heavy filtering)
       * @note tau = 0.5s gives alpha ≈ 0.4 (moderate filtering)
       */
      void setFilterTimeConstant(float tau, float sampleTime) {
        if (tau > 0 && sampleTime > 0) {
          _filterAlpha = sampleTime / (tau + sampleTime);
        } else {
          _filterAlpha = 1.0f; // No filtering
        }
      }

      // Set all them together and reset
      void setTunings(float kp, float ki, float kd) {
        _kp = kp;
        _ki = ki;
        _kd = kd;
      }

      /**
       * @brief Set the output limits for the PID controller to clamp the output value
       * and to prevent integral windup if integral correction mode is CLAMP.
       * @param min Minimum output value
       * @param max Maximum output value
       */
      void setOutputLimits(float min, float max) {
        if (min >= max) {
          unsetOutputLimits();
          return;
        }
        _outputMin = min;
        _outputMax = max;
        _iTerm = _clamp(_iTerm);
        _lastOutput = _clamp(_lastOutput);
      }

      /**
       * @brief Unset output limits, allowing the PID output to take any value.
       * @note This will disable integral windup protection even if integral correction mode is CLAMP.
       */
      void unsetOutputLimits() {
        _outputMin = 0;
        _outputMax = 0;
      }

      /**
       * @brief Reset the PID controller state, including the last input, last error, and integral sum.
       * @brief Optionally set an initial output value to start from. In that case, the integral term and output will be initialized to that value.
       * @brief Setting an initial value can help to prevent large initial overshoots by starting the controller closer to the expected output.
       * @param initialOutput The initial output value to set (default is NAN).
       */
      void reset(float initialOutput = NAN) {
        if (isnan(initialOutput)) {
          _lastOutput = NAN;
          _iTerm = 0;
        } else {
          _lastOutput = _clamp(initialOutput);
          _iTerm = _lastOutput;
        }
        _pTerm = 0;
        _dTerm = 0;
        _lastInput = NAN;
        _lastTime = 0;
        _feed = 0;
      }

      /**
       * @brief Pause the PID controller.
       * @note This will stop the PID computation and hold the last output value.
       */
      void pause() { _enabled = false; }

      /**
       * @brief Resume the PID controller.
       * @note This will allow the PID computation to continue from the last state.
       */
      void resume() {
        _enabled = true;
      }

      /**
       * @brief Resume the PID controller and set an initial output value.
       * @brief This can help to prevent large initial overshoots by starting the controller closer to the expected output.
       * @param initialOutput The initial output value to set.
       */
      void resume(float initialOutput) {
        if (!_enabled) {
          reset(initialOutput);
          _enabled = true;
        }
      }

      /**
       * @brief Compute the PID output value based on the current input.
       * @param input The current input value to the PID controller.
       * @return The computed PID output value, constrained within the output limits if set.
       * @note If the PID controller is paused, this function will return the last output value
       */
      float compute(float input) {
        if (!_enabled)
          return _lastOutput;

        // Apply first-order lag filter (exponential smoothing) to input
        if (!isnan(_lastInput)) {
          input = _filterAlpha * input + (1.0f - _filterAlpha) * _lastInput;
        }

        float kp = _kp;
        float ki = _ki;
        float kd = _kd;

        // compute error and error delta
        const float error = _setpoint - input;
        const float dError = isnan(_lastInput) ? 0 : _lastInput - input;

        // when in reverse mode, invert the gains
        if (_reverse) {
          kp = -kp;
          ki = -ki;
          kd = -kd;
        }

        // time sampling: pre-adjust gains for efficiency (instead of multiplying/dividing in each calculation)
        if (_timeSampling && _lastTime) {
          // time difference in microseconds since last computation, converted to fraction of seconds
          const float dt = static_cast<float>(micros() - _lastTime) / 1000000.0f;
          if (dt > 0) {
            ki = ki * dt;
            kd = kd / dt;
          }
        }

        // calculate proportional term
        switch (_pMode) {
          case ProportionalMode::ON_ERROR:
            // traditional method: proportional term is based on error
            _pTerm = kp * error;
            break;
          case ProportionalMode::ON_INPUT:
            // proportional on measurement, i.e. accumulate the proportional term based on input changes
            _pTerm += kp * dError;
            break;
          default:
            assert(false);
            break;
        }

        // calculate integral term and integrate over time
        _iTerm += ki * error;

        // clamp integral if needed to prevent windup
        if (_icMode == IntegralCorrectionMode::CLAMP) {
          _iTerm = _clamp(_iTerm);
        }

        // calculate derivative term
        _dTerm = kd * dError;

        // calculate output and clamp to output limits
        _lastOutput = _clamp(_pTerm + _iTerm + _dTerm + _feed);

        // remember some values for next time
        _lastInput = input;
        _lastTime = micros();

        return _lastOutput;
      }

#ifdef MYCILA_JSON_SUPPORT
      void toJson(const JsonObject& root) const {
        root["pMode"] = _pMode == ProportionalMode::ON_ERROR ? "error" : "input";
        root["icMode"] = _icMode == IntegralCorrectionMode::OFF ? "off" : "clamp";
        root["reverse"] = _reverse;
        root["time_sampling"] = _timeSampling;
        root["enabled"] = _enabled;
        root["setpoint"] = _setpoint;
        root["kp"] = _kp;
        root["ki"] = _ki;
        root["kd"] = _kd;
        root["output_min"] = _outputMin;
        root["output_max"] = _outputMax;
        root["filter_alpha"] = _filterAlpha;
        root["pTerm"] = _pTerm;
        root["iTerm"] = _iTerm;
        root["dTerm"] = _dTerm;
        root["feed"] = _feed;
        root["last_input"] = _lastInput;
        root["last_output"] = _lastOutput;
        root["last_time"] = _lastTime;
      }
#endif

    private:
      ProportionalMode _pMode = ProportionalMode::ON_ERROR;
      IntegralCorrectionMode _icMode = IntegralCorrectionMode::CLAMP;
      bool _enabled = true;
      bool _reverse = false;
      bool _timeSampling = false;

      float _setpoint = 0;
      float _kp = 0;
      float _ki = 0;
      float _kd = 0;
      float _outputMin = 0;
      float _outputMax = 0;

      float _lastInput = NAN;
      float _lastOutput = NAN;

      float _pTerm = 0;
      float _iTerm = 0;
      float _dTerm = 0;
      float _feed = 0;
      uint32_t _lastTime = 0;

      float _filterAlpha = 1.0f; // 1.0 = no filtering (default)

      inline float _clamp(float value) { return _outputMin == _outputMax ? value : constrain(value, _outputMin, _outputMax); }
  };

} // namespace Mycila
