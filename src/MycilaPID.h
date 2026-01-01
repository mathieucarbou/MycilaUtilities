// SPDX-License-Identifier: MIT
/*
 * Copyright (C) Mathieu Carbou
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
      float getError() const { return _lastError; }

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
       * @brief Get the PID start time (in milliseconds).
       */
      uint32_t getStartTime() const { return _startTime; }

      /**
       * @brief Get the last time (in milliseconds) when compute() was called.
       * @return The last time in milliseconds, or 0 if compute() has never been called.
       */
      uint32_t getLastTime() const { return _lastTime; }

      /**
       * @brief Get the running time of the PID (in milliseconds).
       */
      uint32_t getElapsedTime() const { return _lastTime > _startTime ? _lastTime - _startTime : 0; }

      /**
       * @brief Get the Integral of Absolute Error (IAE)
       * @brief Standard performance metric: lower is better
       * @return Cumulative absolute error
       */
      float getIAE() const { return _iae; }
      /**
       * @brief Get the Integral of Squared Error (ISE)
       * @brief Penalizes large errors more heavily: lower is better
       * @return Cumulative squared error
       */
      float getISE() const { return _ise; }
      /**
       * @brief Get the Integral of Time-weighted Absolute Error (ITAE)
       * @brief Penalizes persistent errors: lower is better
       * @return Cumulative time-weighted absolute error
       */
      float getITAE() const { return _itae; }
      /**
       * @brief Get the average absolute error
       * @brief Useful for understanding typical deviation
       * @return Average absolute error
       */
      float getAverageAbsoluteError() const {
        float elapsed = getElapsedTime() / 1000.0f;
        return elapsed > 0 ? _iae / elapsed : NAN;
      }
      /**
       * @brief Get the Root Mean Square Error (RMSE)
       * @brief Standard deviation of error, useful for comparing tunings
       * @return RMS error
       */
      float getRMSE() const {
        float elapsed = getElapsedTime() / 1000.0f;
        return elapsed > 0 ? sqrt(_ise / elapsed) : NAN;
      }

      // Setters

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
        resetTerms();
        resetMetrics();

        if (isnan(initialOutput)) {
          _lastOutput = NAN;
        } else {
          _lastOutput = _clamp(initialOutput);
          _iTerm = _lastOutput;
        }
        _feed = 0;
      }

      /**
       * @brief Reset only the PID terms (P, I, D) without affecting the last input or output.
       */
      void resetTerms() {
        _pTerm = 0;
        _iTerm = 0;
        _dTerm = 0;
      }

      /**
       * @brief Reset all performance metrics
       * @brief Call this when starting a new control session or after tuning changes
       */
      void resetMetrics() {
        _startTime = 0;
        _lastTime = 0;
        _iae = 0;
        _ise = 0;
        _itae = 0;
        _lastError = NAN;
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

        const uint32_t now = millis();

        // PID start
        if (!_startTime) {
          _startTime = now;
        }

        // compute time difference in seconds since last computation and eventually set a minimal dt
        const float dt = _lastTime ? (now - _lastTime) / 1000.0f : 0.0f;

        // Apply first-order lag filter (exponential smoothing) to input
        if (isnan(_lastInput)) {
          _lastInput = input;
        } else {
          input = _filterAlpha * input + (1.0f - _filterAlpha) * _lastInput;
        }

        // compute error and delta
        const float error = _setpoint - input;
        const float dError = _lastInput - input;

        float kp = _kp;
        float ki = _ki;
        float kd = _kd;

        // when in reverse mode, invert the gains
        if (_reverse) {
          kp = -kp;
          ki = -ki;
          kd = -kd;
        }

        // time sampling: pre-adjust gains for efficiency (instead of multiplying/dividing in each calculation)
        if (_timeSampling && dt > 0) {
          ki = ki * dt;
          kd = kd / dt;
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

          // clamp proportional term to max output limit in both directions
          if (_pMode == ProportionalMode::ON_INPUT) {
            if (_pTerm > _outputMax)
              _pTerm = _outputMax;
            else if (_pTerm < -_outputMax)
              _pTerm = -_outputMax;
          }
        }

        // calculate derivative term
        _dTerm = kd * dError;

        // calculate output and clamp to output limits
        _lastOutput = _clamp(_pTerm + _iTerm + _dTerm + _feed);

        // calculate statistics
        if (!isnan(_lastError)) {
          const float avgAbsError = (abs(_lastError) + abs(error)) / 2.0f;
          const float avgSqError = (_lastError * _lastError + error * error) / 2.0f;
          _iae += avgAbsError * dt;
          _ise += avgSqError * dt;
          _itae += avgAbsError * dt * (now - _startTime) / 1000.0f;
        }

        // remember values for next time
        _lastInput = input;
        _lastError = error;
        _lastTime = now;

        return _lastOutput;
      }

#ifdef MYCILA_JSON_SUPPORT
      void toJson(const JsonObject& root) const {
        JsonObject config = root["config"].to<JsonObject>();
        config["enabled"] = _enabled;
        config["pMode"] = _pMode == ProportionalMode::ON_ERROR ? "error" : "input";
        config["icMode"] = _icMode == IntegralCorrectionMode::OFF ? "off" : "clamp";
        config["reverse"] = _reverse;
        config["time_sampling"] = _timeSampling;
        config["setpoint"] = _setpoint;
        config["kp"] = _kp;
        config["ki"] = _ki;
        config["kd"] = _kd;
        config["filter_alpha"] = _filterAlpha;
        config["output"]["min"] = _outputMin;
        config["output"]["max"] = _outputMax;
        JsonObject runtime = root["runtime"].to<JsonObject>();
        runtime["start_time"] = _startTime;
        runtime["last_time"] = _lastTime;
        runtime["elapsed"] = getElapsedTime();
        runtime["last_input"] = _lastInput;
        runtime["last_output"] = _lastOutput;
        runtime["last_error"] = _lastError;
        runtime["pTerm"] = _pTerm;
        runtime["iTerm"] = _iTerm;
        runtime["dTerm"] = _dTerm;
        runtime["feed"] = _feed;
        JsonObject stats = root["stats"].to<JsonObject>();
        stats["iae"] = _iae;
        stats["ise"] = _ise;
        stats["itae"] = _itae;
        stats["rmse"] = getRMSE();
        stats["avg_err"] = getAverageAbsoluteError();
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
      float _filterAlpha = 1.0f; // 1.0 = no filtering (default)

      uint32_t _startTime = 0;
      uint32_t _lastTime = 0;
      float _lastInput = NAN;
      float _lastOutput = NAN;
      float _lastError = NAN;
      float _pTerm = 0;
      float _iTerm = 0;
      float _dTerm = 0;
      float _feed = 0;

      float _iae = 0;  // Integral of Absolute Error
      float _ise = 0;  // Integral of Squared Error
      float _itae = 0; // Integral of Time-weighted Absolute Error

      inline float _clamp(float value) { return _outputMin == _outputMax ? value : constrain(value, _outputMin, _outputMax); }
  };

} // namespace Mycila
