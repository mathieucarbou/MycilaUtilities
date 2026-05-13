/**
 * MycilaPID - JavaScript implementation
 * Based on the C++ implementation from MycilaUtilities
 * Copyright (C) Mathieu Carbou
 */

class PID {
  constructor() {
    // Modes
    this.ProportionalMode = { ON_ERROR: 'error', ON_INPUT: 'input' };
    this.DerivativeMode = { ON_ERROR: 'error', ON_INPUT: 'input' };
    this.IntegralCorrectionMode = { OFF: 'off', CLAMP: 'clamp' };

    // Configuration
    this._pMode = this.ProportionalMode.ON_INPUT;
    this._dMode = this.DerivativeMode.ON_ERROR;
    this._icMode = this.IntegralCorrectionMode.CLAMP;
    this._enabled = true;
    this._reverse = false;
    this._timeSampling = false;

    // Tuning parameters
    this._setpoint = 0;
    this._kp = 0;
    this._ki = 0;
    this._kd = 0;
    this._outputMin = NaN;
    this._outputMax = NaN;
    this._filterAlpha = 1.0;

    // State
    this._lastInput = NaN;
    this._lastOutput = NaN;
    this._lastError = NaN;
    this._pTerm = NaN;
    this._iTerm = NaN;
    this._dTerm = NaN;
    this._feed = 0;
    this._startTime = 0;
    this._lastTime = 0;
  }

  // Getters
  getProportionalMode() { return this._pMode; }
  getDerivativeMode() { return this._dMode; }
  getIntegralCorrectionMode() { return this._icMode; }
  isReverse() { return this._reverse; }
  isEnabled() { return this._enabled; }
  isTimeSampling() { return this._timeSampling; }
  getSetpoint() { return this._setpoint; }
  getKp() { return this._kp; }
  getKi() { return this._ki; }
  getKd() { return this._kd; }
  getOutputMin() { return this._outputMin; }
  getOutputMax() { return this._outputMax; }
  getInput() { return this._lastInput; }
  getOutput() { return this._lastOutput; }
  getPTerm() { return this._pTerm; }
  getITerm() { return this._iTerm; }
  getDTerm() { return this._dTerm; }
  getFeedForward() { return this._feed; }
  getFilterAlpha() { return this._filterAlpha; }
  getStartTime() { return this._startTime; }
  getLastTime() { return this._lastTime; }

  // Setters
  setProportionalMode(mode) { this._pMode = mode; }
  setDerivativeMode(mode) { this._dMode = mode; }
  setIntegralCorrectionMode(mode) { this._icMode = mode; }
  setReverse(reverse) { this._reverse = reverse; }
  setTimeSampling(timeSampling) { this._timeSampling = timeSampling; }
  setSetpoint(setpoint) { this._setpoint = setpoint; }
  setKp(kp) { this._kp = kp; }
  setKi(ki) { this._ki = ki; }
  setKd(kd) { this._kd = kd; }
  setFeedForward(feedForward) { this._feed = feedForward; }
  setFilterAlpha(alpha) { this._filterAlpha = this._clamp(alpha, 0.0, 1.0); }

  setFilterTimeConstant(tau, sampleTime) {
    if (tau > 0 && sampleTime > 0) {
      this._filterAlpha = sampleTime / (tau + sampleTime);
    } else {
      this._filterAlpha = 1.0;
    }
  }

  setTunings(kp, ki, kd) {
    this._kp = kp;
    this._ki = ki;
    this._kd = kd;
  }

  setOutputLimits(min, max) {
    this._outputMin = min;
    this._outputMax = max;
    this._lastOutput = this._clamp(this._lastOutput);
    this._iTerm = this._clamp(this._lastOutput - this._pTerm - this._dTerm - this._feed);
  }

  unsetOutputLimits() {
    this.setOutputLimits(NaN, NaN);
  }

  reset(initialOutput = NaN) {
    this.resetTerms();
    this._startTime = 0;
    this._lastTime = 0;
    this._lastError = NaN;

    if (isNaN(initialOutput)) {
      this._lastOutput = NaN;
    } else {
      this._lastOutput = this._clamp(initialOutput);
      this._iTerm = this._lastOutput;
    }
    this._feed = 0;
  }

  resetTerms() {
    this._pTerm = NaN;
    this._iTerm = NaN;
    this._dTerm = NaN;
  }

  pause() {
    this._enabled = false;
  }

  resume(initialOutput = NaN) {
    if (!this._enabled) {
      if (!isNaN(initialOutput)) {
        this.reset(initialOutput);
      }
      this._enabled = true;
    }
  }

  compute(input) {
    if (!this._enabled) {
      return this._lastOutput;
    }

    const now = performance.now();

    if (!this._startTime) {
      this._startTime = now;
    }

    const dt = this._lastTime ? (now - this._lastTime) / 1000.0 : 0.0;

    if (isNaN(this._lastInput)) {
      this._lastInput = input;
    } else {
      input = this._filterAlpha * input + (1.0 - this._filterAlpha) * this._lastInput;
    }

    let kp = this._kp;
    let ki = this._ki;
    let kd = this._kd;

    const error = this._setpoint - input;
    const dError = isNaN(this._lastError) ? 0 : error - this._lastError;
    const dInput = input - this._lastInput;

    // When in reverse mode, invert the gains
    if (this._reverse) {
      kp = -kp;
      ki = -ki;
      kd = -kd;
    }

    // Time sampling
    if (this._timeSampling && dt > 0) {
      ki = ki * dt;
      kd = kd / dt;
    }

    // Calculate proportional term
    switch (this._pMode) {
      case this.ProportionalMode.ON_ERROR:
        // Traditional method: proportional term is based on error
        this._pTerm = kp * error;
        break;
      case this.ProportionalMode.ON_INPUT:
        this._pTerm = this._zeroNan(this._pTerm) + kp * dInput;
        break;
    }

    this._iTerm = this._zeroNan(this._iTerm) + ki * error;

    // Calculate derivative term
    switch (this._dMode) {
      case this.DerivativeMode.ON_ERROR:
        this._dTerm = kd * dError;
        break;
      case this.DerivativeMode.ON_INPUT:
        this._dTerm = -kd * dInput;
        break;
    }

    // Calculate output and clamp to output limits
    this._lastOutput = this._clamp(this._pTerm + this._iTerm + this._dTerm + this._feed);

    if (this._icMode === this.IntegralCorrectionMode.CLAMP) {
      this._iTerm = this._lastOutput - this._pTerm - this._dTerm;
    }

    // Remember values for next time
    this._lastInput = input;
    this._lastError = error;
    this._lastTime = now;

    return this._lastOutput;
  }

  _clamp(value, min = this._outputMin, max = this._outputMax) {
    if (Number.isNaN(value) || (Number.isNaN(min) && Number.isNaN(max))) {
      return value;
    }
    if (!Number.isNaN(min) && value < min) {
      return min;
    }
    if (!Number.isNaN(max) && value > max) {
      return max;
    }
    return value;
  }

  _zeroNan(value) {
    return Number.isNaN(value) ? 0 : value;
  }

  toJSON() {
    return {
      pMode: this._pMode,
      dMode: this._dMode,
      icMode: this._icMode,
      reverse: this._reverse,
      enabled: this._enabled,
      timeSampling: this._timeSampling,
      setpoint: this._setpoint,
      kp: this._kp,
      ki: this._ki,
      kd: this._kd,
      filter_alpha: this._filterAlpha,
      output_min: this._outputMin,
      output_max: this._outputMax,
      start_time: this._startTime,
      pTerm: this._pTerm,
      iTerm: this._iTerm,
      dTerm: this._dTerm,
      feed: this._feed,
      last_input: this._lastInput,
      last_output: this._lastOutput,
      last_time: this._lastTime
    };
  }
}
