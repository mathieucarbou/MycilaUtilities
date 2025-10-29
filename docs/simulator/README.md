# PID Simulator - Web Version

A standalone web-based PID controller simulator that demonstrates solar power excess diversion to a load.

## Overview

This is a JavaScript implementation of the PID Simulator originally written in C++ for ESP32. It runs entirely in the browser without requiring any backend server or hardware.

## Features

- **Real-time PID Simulation**: Watch the PID controller manage solar power diversion in real-time
- **Interactive Charts**: 7 separate charts showing Solar, Grid, pTerm, iTerm, dTerm, Output, and Load
- **Full PID Control**: Adjust all PID parameters (Kp, Ki, Kd, setpoint)
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
