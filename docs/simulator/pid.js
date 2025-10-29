/**
 * MycilaPID - JavaScript implementation
 * Based on the C++ implementation from MycilaUtilities
 * Copyright (C) 2023-2025 Mathieu Carbou
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
    this._outputMin = 0;
    this._outputMax = 0;

    // State
    this._lastInput = NaN;
    this._lastOutput = NaN;
    this._lastError = NaN;
    this._pTerm = 0;
    this._iTerm = 0;
    this._dTerm = 0;
    this._feed = 0;
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

  setTunings(kp, ki, kd) {
    this._kp = kp;
    this._ki = ki;
    this._kd = kd;
  }

  setOutputLimits(min, max) {
    if (min >= max) {
      this.unsetOutputLimits();
      return;
    }
    this._outputMin = min;
    this._outputMax = max;
    this._iTerm = this._clamp(this._iTerm);
    this._lastOutput = this._clamp(this._lastOutput);
  }

  unsetOutputLimits() {
    this._outputMin = 0;
    this._outputMax = 0;
  }

  reset(initialOutput = NaN) {
    if (isNaN(initialOutput)) {
      this._lastOutput = NaN;
      this._iTerm = 0;
    } else {
      this._lastOutput = this._clamp(initialOutput);
      this._iTerm = this._lastOutput;
    }
    this._pTerm = 0;
    this._dTerm = 0;
    this._lastInput = NaN;
    this._lastError = NaN;
    this._lastTime = 0;
    this._feed = 0;
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

    let kp = this._kp;
    let ki = this._ki;
    let kd = this._kd;

    // Input delta
    const dInput = isNaN(this._lastInput) ? 0 : input - this._lastInput;

    // Compute error and error delta
    const error = this._setpoint - input;
    const dError = isNaN(this._lastError) ? 0 : error - this._lastError;

    // When in reverse mode, invert the gains
    if (this._reverse) {
      kp = -kp;
      ki = -ki;
      kd = -kd;
    }

    // Time sampling
    if (this._timeSampling && this._lastTime) {
      // Time difference in milliseconds since last computation, converted to fraction of seconds
      const now = performance.now();
      const dt = (now - this._lastTime) / 1000.0;
      if (dt > 0) {
        ki = ki * dt;
        kd = kd / dt;
      }
    }

    // Calculate proportional term
    switch (this._pMode) {
      case this.ProportionalMode.ON_ERROR:
        // Traditional method: proportional term is based on error
        this._pTerm = kp * error;
        break;
      case this.ProportionalMode.ON_INPUT:
        // Proportional on measurement
        this._pTerm -= kp * dInput;
        break;
    }

    // Calculate integral term
    this._iTerm += ki * error;

    // Clamp integral if needed
    if (this._icMode === this.IntegralCorrectionMode.CLAMP) {
      this._iTerm = this._clamp(this._iTerm);
    }

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

    // Remember values for next time
    this._lastInput = input;
    this._lastError = error;
    this._lastTime = performance.now();

    return this._lastOutput;
  }

  _clamp(value) {
    if (this._outputMin === this._outputMax) {
      return value;
    }
    return Math.max(this._outputMin, Math.min(this._outputMax, value));
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
      output_min: this._outputMin,
      output_max: this._outputMax,
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
