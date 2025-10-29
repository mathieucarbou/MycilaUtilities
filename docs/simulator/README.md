# PID Simulator - Web Version

A standalone web-based PID controller simulator that demonstrates solar power excess diversion to a load.

## Overview

This is a JavaScript implementation of the PID Simulator originally written in C++ for ESP32. It runs entirely in the browser without requiring any backend server or hardware.

## Features

- **Real-time PID Simulation**: Watch the PID controller manage solar power diversion in real-time
- **Interactive Charts**: 7 separate charts showing Solar, Grid, pTerm, iTerm, dTerm, Output, and Load
- **Performance Metrics**: Live analysis of PID controller effectiveness with 8 key metrics including damping ratio
- **Full PID Control**: Adjust all PID parameters (Kp, Ki, Kd, setpoint, feed-forward)
- **Advanced Settings**: Configure reverse mode, time sampling, output limits
- **Controller Modes**: 
  - Proportional Mode (ON_ERROR, ON_INPUT)
  - Derivative Mode (ON_ERROR, ON_INPUT)
  - Integral Correction Mode (OFF, CLAMP)
- **Simulation Controls**: Pause, Resume, Reset the simulation

## Files

- `index.html` - Main HTML page with UI layout
- `style.css` - Styling for the simulator interface
- `pid.js` - JavaScript implementation of the MycilaPID controller
- `simulator.js` - Simulation logic and chart management

## Usage

Simply open `index.html` in a modern web browser. No build process or server required.

### Controls

1. **PID Parameters**:
   - **Kp**: Proportional gain
   - **Ki**: Integral gain
   - **Kd**: Derivative gain
   - **Setpoint**: Target value (typically 0 for grid balance)

2. **Controller Settings**:
   - **Reverse Mode**: Inverts the controller action
   - **Time Sampling**: Accounts for variable time intervals
   - **Output Min/Max**: Limits for the controller output

3. **Advanced Modes**:
   - **Integral Correction**: CLAMP (prevents windup) or OFF
   - **Proportional Mode**: ON_ERROR (traditional) or ON_INPUT (measurement-based)
   - **Derivative Mode**: ON_ERROR (traditional) or ON_INPUT (measurement-based)

4. **Buttons**:
   - **Apply Settings**: Apply parameter changes
   - **Reset**: Reset the simulation and PID state
   - **Pause/Resume**: Control simulation execution

### Chart Interaction

Click on any chart title to show/hide that specific chart.

### Performance Metrics

The simulator provides real-time metrics to evaluate PID controller performance:

1. **Error (Avg)**: Average absolute distance from setpoint *when actively diverting* - lower is better. Indicates control accuracy when excess solar is available.
2. **Error (RMS)**: Root Mean Square error during active control - measures tracking performance when diverting power.
3. **Stability**: Standard deviation of errors - shows how consistent the controller is. Lower values mean more stable control.
4. **Damping Ratio (ζ)**: Measures how oscillations decay in the system. Calculated using the logarithmic decrement method from consecutive oscillation peaks.
   - **ζ < 0.4**: Underdamped (too much oscillation, reduce Kp or increase Kd)
   - **ζ ≈ 0.6-0.8**: Well-damped (optimal - fast response with minimal oscillations)
   - **ζ > 1.0**: Overdamped (slow response, increase Kp or Ki)
5. **Response Time**: Time to reach within ±20W of setpoint, measured from when the controller starts diverting power (output > 0). Measures reaction speed when solar excess becomes available.
6. **Settling Time**: Time to stabilize within ±10W of setpoint, measured from when the controller starts diverting power. Indicates when the system reaches steady state and maintains stable control.
7. **Oscillations**: Number of setpoint crossings during active control - too many indicates unstable tuning.
8. **Control Effort**: Average output magnitude - shows how hard the controller works. Lower effort with good performance is ideal.

**Important for Solar Diversion**: Metrics are calculated intelligently for solar diversion scenarios:
- When PID output is ≤0 (no excess solar), being above setpoint is *expected* and doesn't count as an error
- Metrics focus on performance when the controller is *actively diverting* (output > 0)
- This prevents false "poor performance" readings when there's simply no solar excess to divert

**Color coding**: Green = Good, Yellow = Warning, Red = Needs improvement

**Tip**: After changing PID parameters, click "Apply Settings" to reset metrics and get fresh measurements for comparison.

## Simulation Details

The simulator models a solar power diversion system where:

- **Solar**: Random solar production (0-1000W) with gradual variations
- **Grid**: Grid power consumption/export
- **Load**: Divertible load (0-2000W) controlled by PID output
- **Random variations**: Simulates real-world noise and sudden load changes

The PID controller attempts to maintain the grid setpoint (typically 0W) by adjusting the diverted load based on available solar excess.

## Implementation

The JavaScript PID implementation closely follows the C++ `MycilaPID` class, including:

- All PID modes and settings
- Anti-windup protection (CLAMP mode)
- Time-sampling support
- Reverse action
- Feed-forward support (defaults to 0)

## Browser Compatibility

Requires a modern browser with support for:
- ES6 JavaScript
- Canvas API
- Chart.js 4.x

Tested on:
- Chrome/Edge 90+
- Firefox 88+
- Safari 14+

## Learn More

- [MycilaPID Documentation](../pid.md) - Detailed PID controller documentation
- [ESP32 Implementation](../../examples/PIDSimulator/PIDSimulator.ino) - Original C++ version

## License

Copyright (C) 2023-2025 Mathieu Carbou  
Licensed under MIT License
