// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2024 Mathieu Carbou
 */
#pragma once

#ifdef MYCILA_JSON_SUPPORT
  #include <ArduinoJson.h>
#endif

#include <Print.h>

namespace Mycila {
  // Inspired by:
  // - https://github.com/Dlloydev/QuickPID
  // - https://github.com/br3ttb/Arduino-PID-Library
  class PID {
    public:
      enum class ProportionalMode {
        // proportional term computed from the error
        P_ON_ERROR = 1,
        // proportional term computed from the input
        P_ON_INPUT = 2,
        // proportional term computed from the error and input 50% each
        P_ON_BOTH = 3,
      };

      enum class DerivativeMode {
        // derivative term computed from the error
        D_ON_ERROR = 1,
        // derivative term computed from the input
        D_ON_INPUT = 2,
      };

      //  integral anti-windup
      enum class IntegralCorrectionMode {
        // off
        IC_OFF = 0,
        // clamp the integral term between the limits
        IC_CLAMP = 1,
        // provide some integral correction, prevents deep saturation and reduces overshoot
        IC_ADVANCED = 2,
      };

    public:
      void setProportionalMode(ProportionalMode mode) { _pMode = mode; }
      void setDerivativeMode(DerivativeMode mode) { _dMode = mode; }
      void setIntegralCorrectionMode(IntegralCorrectionMode mode) { _icMode = mode; }
      void setReverse(bool reverse) { _reverse = reverse; }

      void unsetOutputLimits() {
        _outputMin = 0;
        _outputMax = 0;
      }

      // If min == max, output or sum is not clamped
      void setOutputLimits(float min, float max) {
        if (min > max)
          return;
        _outputMin = min;
        _outputMax = max;
        _sum = constrain(_sum, _outputMin, _outputMax);
        _output = constrain(_output, _outputMin, _outputMax);
      }

      // PID target
      void setSetPoint(float setPoint) { _setPoint = setPoint; }

      // PID proportional gain
      void setKp(float kp) { _kp = kp; }
      // PID integral gain
      void setKi(float ki) { _ki = ki; }
      // PID derivative gain
      void setKd(float kd) { _kd = kd; }

      // Set all them together and reset
      void setTunings(float kp, float ki, float kd) {
        _kp = kp;
        _ki = ki;
        _kd = kd;
      }

      // Reset PID state but keep its settings
      void reset() {
        _input = 0;
        _error = 0;
        _pTerm = 0;
        _iTerm = 0;
        _dTerm = 0;
        _sum = constrain(0, _outputMin, _outputMax);
        _output = constrain(0, _outputMin, _outputMax);
      }

      float compute(float input) {
        const float dInput = _reverse ? _input - input : input - _input;
        const float error = _reverse ? input - _setPoint : _setPoint - input;
        const float dError = error - _error;

        float peTerm = _kp * error;
        float pmTerm = _kp * dInput;
        switch (_pMode) {
          case ProportionalMode::P_ON_ERROR:
            pmTerm = 0;
            break;
          case ProportionalMode::P_ON_INPUT:
            peTerm = 0;
            break;
          case ProportionalMode::P_ON_BOTH:
            peTerm *= 0.5f;
            pmTerm *= 0.5f;
            break;
          default:
            assert(false);
            break;
        }

        // pTerm
        _pTerm = peTerm - pmTerm;

        // iTerm
        _iTerm = _ki * error;

        // anti-windup
        if (_icMode == IntegralCorrectionMode::IC_ADVANCED && _ki) {
          const float iTermOut = _pTerm + _ki * (_iTerm + error);
          if ((iTermOut > _outputMax && dError > 0) || (iTermOut < _outputMin && dError < 0)) {
            _iTerm = constrain(iTermOut, -_outputMax, _outputMax);
          }
        }

        // integral sum
        _sum = _icMode == IntegralCorrectionMode::IC_OFF ? (_sum + _iTerm - pmTerm) : constrain(_sum + _iTerm - pmTerm, _outputMin, _outputMax);

        // dTerm
        switch (_dMode) {
          case DerivativeMode::D_ON_ERROR:
            _dTerm = _kd * dError;
            break;
          case DerivativeMode::D_ON_INPUT:
            _dTerm = -_kd * dInput;
            break;
          default:
            assert(false);
            break;
        }

        _output = constrain(_sum + peTerm + _dTerm, _outputMin, _outputMax);

        this->_input = input;
        this->_error = error;

        return _output;
      }

      ProportionalMode getProportionalMode() const { return _pMode; }
      DerivativeMode getDerivativeMode() const { return _dMode; }
      IntegralCorrectionMode getIntegralCorrectionMode() const { return _icMode; }
      bool isReversed() const { return _reverse; }

      float getSetPoint() const { return _setPoint; }
      float getKp() const { return _kp; }
      float getKi() const { return _ki; }
      float getKd() const { return _kd; }
      float getOutputMin() const { return _outputMin; }
      float getOutputMax() const { return _outputMax; }

      float getInput() const { return _input; }
      float getOutput() const { return _output; }
      float getError() const { return _error; }
      float getPTerm() const { return _pTerm; }
      float getITerm() const { return _iTerm; }
      float getDTerm() const { return _dTerm; }
      float getSum() const { return _sum; }

      void dump(Print& printer) { // NOLINT (runtime/references)
        printer.print("PID(");
        printer.print(_input);
        printer.print(")=");
        printer.print(_output);
        printer.print(" (pTerm=");
        printer.print(_pTerm);
        printer.print(", iTerm=");
        printer.print(_iTerm);
        printer.print(", dTerm=");
        printer.print(_dTerm);
        printer.print(")");
      }

#ifdef MYCILA_JSON_SUPPORT
      void toJson(const JsonObject& root) const {
        root["pMode"] = static_cast<int>(_pMode);
        root["dMode"] = static_cast<int>(_dMode);
        root["icMode"] = static_cast<int>(_icMode);
        root["reverse"] = _reverse;
        root["setpoint"] = _setPoint;
        root["kp"] = _kp;
        root["ki"] = _ki;
        root["kd"] = _kd;
        root["outputMin"] = _outputMin;
        root["outputMax"] = _outputMax;
        root["input"] = _input;
        root["output"] = _output;
        root["pTerm"] = _pTerm;
        root["iTerm"] = _iTerm;
        root["dTerm"] = _dTerm;
        root["sum"] = _sum;
      }
#endif

    private:
      ProportionalMode _pMode = ProportionalMode::P_ON_ERROR;
      DerivativeMode _dMode = DerivativeMode::D_ON_INPUT;
      IntegralCorrectionMode _icMode = IntegralCorrectionMode::IC_ADVANCED;
      bool _reverse = false;
      float _setPoint = 0;
      float _kp = 0;
      float _ki = 0;
      float _kd = 0;
      float _outputMin = 0;
      float _outputMax = 0;

    private:
      // PID input
      float _input = 0;
      // PID output
      float _output = 0;
      // PID error
      float _error = 0;
      // PID proportional term
      float _pTerm = 0;
      // PID integral term
      float _iTerm = 0;
      // PID derivative term
      float _dTerm = 0;
      // Internal integral sum
      float _sum;
  };

} // namespace Mycila
