/**
 * PID Simulator - Solar Power Diversion Simulator
 * Based on the ESP32 implementation from MycilaUtilities/examples/PIDSimulator
 */

// Configuration
const MAX_POINTS = 100;
const SEND_INTERVAL = 330; // ms to simulate a JSY response rate

// Chart configuration
const CHART_KEYS = ['Solar (W)', 'Grid (W)', 'pTerm (W)', 'iTerm (W)', 'dTerm (W)', 'Output (W)', 'Load (W)'];
const CHART_COLORS = ['#9A6324', '#e6194b', '#3cb44b', '#ffe119', '#4363d8', '#f58231', '#911eb4'];

// Global state
let pid = new PID();
let charts = [];
let simulationInterval = null;
let isPaused = false;

// Simulation state
let grid = 0;              // grid power in watts
let load = 0;              // load where to divert excess power watts
let random_load_high = 0;  // random high load up to 1000W
let random_load_low = 0;   // random low load between -20W and +20W
let random_solar = 0;      // random solar power generation between 0W and 1000W

// Metrics tracking
let metricsData = {
  errors: [],              // Error history (setpoint - input)
  outputs: [],             // Output history
  settledTime: null,       // Time when system settled
  responseTime: null,      // Time to reach 95% of setpoint
  oscillationCount: 0,     // Number of setpoint crossings
  lastErrorSign: null,     // Sign of last error (for oscillation detection)
  firstPeakError: null,    // First peak error for damping calculation
  secondPeakError: null,   // Second peak error for damping calculation
  peakCount: 0,            // Count of peaks detected
  lastPeakTime: 0,         // Time of last peak
  lastActivationTime: null,// Time when controller last became active (output > 0)
  wasInactive: true,       // Track if we were previously inactive
  startTime: Date.now(),   // Simulation start time
  updateInterval: null     // Interval for metrics updates
};

/**
 * Random integer helper (inclusive)
 */
function randomInt(min, max) {
  return Math.floor(Math.random() * (max - min + 1)) + min;
}

/**
 * Constrain value between min and max
 */
function constrain(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

/**
 * Update metrics based on current PID state
 */
function updateMetrics(input, output, setpoint) {
  const error = setpoint - input;
  
  // Only track metrics when actively controlling (output > 0)
  // or when error is negative (below setpoint is expected when not diverting)
  const isActivelyControlling = output > 0;
  
  // Track when controller becomes active (starts diverting)
  if (isActivelyControlling && metricsData.wasInactive) {
    metricsData.lastActivationTime = Date.now();
    metricsData.wasInactive = false;
    // Reset response and settling times when starting a new diversion period
    metricsData.responseTime = null;
    metricsData.settledTime = null;
  } else if (!isActivelyControlling && !metricsData.wasInactive) {
    metricsData.wasInactive = true;
  }
  
  // Only track errors and outputs when actively controlling
  // When not diverting, grid consumption is expected and not an error
  if (isActivelyControlling) {
    metricsData.errors.push(Math.abs(error));
    metricsData.outputs.push(Math.abs(output));
    
    // Keep history limited
    if (metricsData.errors.length > MAX_POINTS) {
      metricsData.errors.shift();
      metricsData.outputs.shift();
    }
  }
  
  // Response time (time to reach within ±20W for setpoint=0, or 5% for non-zero setpoint)
  // Only calculate if we're actively controlling and have a valid activation time
  if (!metricsData.responseTime && isActivelyControlling && metricsData.lastActivationTime) {
    // Use fixed threshold of ±20W for solar diversion (setpoint = 0)
    // Use 5% threshold for non-zero setpoints
    const responseThreshold = pid.getSetpoint() === 0 ? 20 : Math.abs(pid.getSetpoint()) * 0.05;
    if (Math.abs(error) <= responseThreshold) {
      metricsData.responseTime = (Date.now() - metricsData.lastActivationTime) / 1000;
    }
  }
  
  // Settling time (time to stabilize within ±10W for setpoint=0, or 2% for non-zero setpoint)
  // Only calculate if we're actively controlling and have a valid activation time
  if (!metricsData.settledTime && isActivelyControlling && metricsData.lastActivationTime) {
    // Use fixed threshold of ±10W for solar diversion (setpoint = 0)
    // Use 2% threshold for non-zero setpoints
    const settlingThreshold = pid.getSetpoint() === 0 ? 10 : Math.abs(pid.getSetpoint()) * 0.02;
    if (Math.abs(error) <= settlingThreshold) {
      metricsData.settledTime = (Date.now() - metricsData.lastActivationTime) / 1000;
    }
  }
  
  // Oscillation detection (setpoint crossings)
  const errorSign = Math.sign(error);
  if (metricsData.lastErrorSign !== null && errorSign !== 0 && 
      metricsData.lastErrorSign !== errorSign && isActivelyControlling) {
    metricsData.oscillationCount++;
  }
  metricsData.lastErrorSign = errorSign;
  
  // Peak detection for damping ratio calculation
  // Only during active control to avoid false peaks
  if (isActivelyControlling && metricsData.errors.length >= 3) {
    const currentError = Math.abs(error);
    const prevError = Math.abs(metricsData.errors[metricsData.errors.length - 2]);
    const prevPrevError = Math.abs(metricsData.errors[metricsData.errors.length - 3]);
    
    // Detect peak: error was increasing then started decreasing
    const isPeak = prevError > prevPrevError && prevError > currentError && prevError > 10;
    
    if (isPeak) {
      const now = Date.now();
      // Filter out noise - require at least 1 second between peaks
      if (now - metricsData.lastPeakTime > 1000) {
        metricsData.peakCount++;
        
        if (metricsData.firstPeakError === null) {
          metricsData.firstPeakError = prevError;
        } else if (metricsData.secondPeakError === null) {
          metricsData.secondPeakError = prevError;
        } else {
          // Keep tracking most recent two peaks
          metricsData.firstPeakError = metricsData.secondPeakError;
          metricsData.secondPeakError = prevError;
        }
        
        metricsData.lastPeakTime = now;
      }
    }
  }
}

/**
 * Calculate and display metrics
 */
function displayMetrics() {
  if (metricsData.errors.length < 2) {
    return; // Not enough data yet
  }

  // Average Error
  const avgError = metricsData.errors.reduce((sum, e) => sum + Math.abs(e), 0) / metricsData.errors.length;
  document.getElementById('metricAvgError').textContent = avgError.toFixed(1) + ' W';
  document.getElementById('metricAvgError').className = 'metric-value ' + 
    (avgError < 10 ? 'good' : avgError < 50 ? 'warning' : 'bad');

  // RMS Error
  const rmsError = Math.sqrt(metricsData.errors.reduce((sum, e) => sum + e * e, 0) / metricsData.errors.length);
  document.getElementById('metricRmsError').textContent = rmsError.toFixed(1) + ' W';
  document.getElementById('metricRmsError').className = 'metric-value ' + 
    (rmsError < 15 ? 'good' : rmsError < 75 ? 'warning' : 'bad');

  // Stability (standard deviation of recent errors)
  const mean = metricsData.errors.reduce((sum, e) => sum + e, 0) / metricsData.errors.length;
  const variance = metricsData.errors.reduce((sum, e) => sum + Math.pow(e - mean, 2), 0) / metricsData.errors.length;
  const stability = Math.sqrt(variance);
  document.getElementById('metricStability').textContent = stability.toFixed(1) + ' W';
  document.getElementById('metricStability').className = 'metric-value ' + 
    (stability < 20 ? 'good' : stability < 100 ? 'warning' : 'bad');

  // Damping Ratio (ζ) using logarithmic decrement method
  // ζ = ln(x1/x2) / sqrt(4π² + ln²(x1/x2))
  // where x1 and x2 are consecutive peak amplitudes
  if (metricsData.firstPeakError !== null && metricsData.secondPeakError !== null && 
      metricsData.secondPeakError > 0 && metricsData.firstPeakError > metricsData.secondPeakError) {
    const ratio = metricsData.firstPeakError / metricsData.secondPeakError;
    const lnRatio = Math.log(ratio);
    const damping = lnRatio / Math.sqrt(4 * Math.PI * Math.PI + lnRatio * lnRatio);
    
    document.getElementById('metricDamping').textContent = damping.toFixed(3);
    document.getElementById('metricDamping').className = 'metric-value ' + 
      (damping >= 0.6 && damping <= 0.8 ? 'good' : damping >= 0.4 && damping <= 1.0 ? 'warning' : 'bad');
  } else if (metricsData.oscillationCount === 0 && metricsData.errors.length > 50) {
    // No oscillations detected - likely overdamped
    document.getElementById('metricDamping').textContent = '>1.0';
    document.getElementById('metricDamping').className = 'metric-value warning';
  } else {
    document.getElementById('metricDamping').textContent = 'Calculating...';
    document.getElementById('metricDamping').className = 'metric-value';
  }

  // Response Time
  if (metricsData.responseTime !== null) {
    document.getElementById('metricResponseTime').textContent = metricsData.responseTime.toFixed(1) + ' s';
    document.getElementById('metricResponseTime').className = 'metric-value ' + 
      (metricsData.responseTime < 5 ? 'good' : metricsData.responseTime < 15 ? 'warning' : 'bad');
  } else {
    document.getElementById('metricResponseTime').textContent = 'Waiting...';
    document.getElementById('metricResponseTime').className = 'metric-value';
  }

  // Settling Time
  if (metricsData.settledTime !== null) {
    const currentSettlingTime = (Date.now() - metricsData.startTime) / 1000 - metricsData.settledTime;
    document.getElementById('metricSettlingTime').textContent = metricsData.settledTime.toFixed(1) + ' s';
    document.getElementById('metricSettlingTime').className = 'metric-value ' + 
      (metricsData.settledTime < 10 ? 'good' : metricsData.settledTime < 30 ? 'warning' : 'bad');
  } else {
    document.getElementById('metricSettlingTime').textContent = 'Not settled';
    document.getElementById('metricSettlingTime').className = 'metric-value';
  }

  // Oscillations
  document.getElementById('metricOscillations').textContent = metricsData.oscillationCount.toString();
  document.getElementById('metricOscillations').className = 'metric-value ' + 
    (metricsData.oscillationCount < 5 ? 'good' : metricsData.oscillationCount < 20 ? 'warning' : 'bad');

  // Control Effort (average absolute output)
  if (metricsData.outputs.length > 0) {
    const avgOutput = metricsData.outputs.reduce((sum, o) => sum + o, 0) / metricsData.outputs.length;
    document.getElementById('metricControlEffort').textContent = avgOutput.toFixed(1) + ' W';
    document.getElementById('metricControlEffort').className = 'metric-value ' + 
      (avgOutput < 500 ? 'good' : avgOutput < 1500 ? 'warning' : 'bad');
  } else {
    document.getElementById('metricControlEffort').textContent = '--';
    document.getElementById('metricControlEffort').className = 'metric-value';
  }
}

/**
 * Reset metrics
 */
function resetMetrics() {
  metricsData = {
    errors: [],
    outputs: [],
    settledTime: null,
    responseTime: null,
    oscillationCount: 0,
    lastErrorSign: null,
    firstPeakError: null,
    secondPeakError: null,
    peakCount: 0,
    lastPeakTime: 0,
    lastActivationTime: null,
    wasInactive: true,
    startTime: Date.now(),
    updateInterval: metricsData.updateInterval
  };
  
  // Clear display
  ['metricAvgError', 'metricRmsError', 'metricStability', 'metricDamping',
   'metricResponseTime', 'metricSettlingTime', 'metricOscillations', 'metricControlEffort'].forEach(id => {
    document.getElementById(id).textContent = '--';
    document.getElementById(id).className = 'metric-value';
  });
}

/**
 * Create all charts
 */
function createCharts() {
  const container = document.getElementById('charts');
  container.innerHTML = '';
  charts = [];

  for (let i = 0; i < CHART_KEYS.length; i++) {
    // Create wrapper with clickable label
    const wrapper = document.createElement('div');
    wrapper.className = 'chart-wrapper';

    const title = document.createElement('div');
    title.textContent = CHART_KEYS[i];
    title.className = 'chart-title';
    title.id = 'chartTitle' + i;

    // Toggle chart visibility when clicking the title
    title.onclick = () => {
      const c = document.getElementById('chart' + i);
      if (!c) return;
      c.style.display = c.style.display === 'none' ? '' : 'none';
      title.classList.toggle('collapsed', c.style.display === 'none');
    };

    const canvas = document.createElement('canvas');
    canvas.id = 'chart' + i;
    canvas.className = 'chart-canvas';
    canvas.width = 800;
    canvas.height = 120;

    wrapper.appendChild(title);
    wrapper.appendChild(canvas);
    container.appendChild(wrapper);

    const ctx = canvas.getContext('2d');
    const cfg = {
      type: 'line',
      data: {
        labels: [],
        datasets: [{
          label: CHART_KEYS[i],
          data: [],
          borderColor: CHART_COLORS[i],
          backgroundColor: CHART_COLORS[i] + '33',
          fill: false,
          showLine: true,
          spanGaps: true,
          tension: 0.4,
          pointRadius: 0,
          borderWidth: 2
        }]
      },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        plugins: {
          legend: { display: false }
        },
        elements: {
          line: {
            tension: 0.4,
            cubicInterpolationMode: 'monotone'
          }
        },
        scales: {
          x: { display: false },
          y: {
            grid: {
              color: '#e0e0e0'
            },
            ticks: {
              font: { size: 10 }
            }
          }
        },
        layout: {
          padding: 0
        }
      }
    };
    charts.push(new Chart(ctx, cfg));
  }
}

/**
 * Update charts with new data
 */
function updateCharts(values) {
  for (let i = 0; i < values.length && i < charts.length; i++) {
    const chart = charts[i];
    chart.data.labels.push('');
    chart.data.datasets[0].data.push(values[i]);
    
    // Keep only MAX_POINTS
    while (chart.data.labels.length > MAX_POINTS) {
      chart.data.labels.shift();
      chart.data.datasets[0].data.shift();
    }
    chart.update('none'); // Update without animation
  }
}

/**
 * Initialize PID with default values
 */
function initializePID() {
  pid.setReverse(false);
  pid.setTimeSampling(false);
  pid.setOutputLimits(-300, 4000);
  pid.setIntegralCorrectionMode(pid.IntegralCorrectionMode.CLAMP);
  pid.setProportionalMode(pid.ProportionalMode.ON_INPUT);
  pid.setDerivativeMode(pid.DerivativeMode.ON_ERROR);
  pid.setKp(0.1);
  pid.setKi(0.2);
  pid.setKd(0.05);
  pid.setSetpoint(0);
  
  // Sync UI controls with PID state
  syncUIWithPID();
}

/**
 * Sync UI controls with current PID state
 */
function syncUIWithPID() {
  document.getElementById('kp').value = pid.getKp();
  document.getElementById('ki').value = pid.getKi();
  document.getElementById('kd').value = pid.getKd();
  document.getElementById('setpoint').value = pid.getSetpoint();
  document.getElementById('feedForward').value = pid.getFeedForward();
  document.getElementById('reverse').checked = pid.isReverse();
  document.getElementById('timeSampling').checked = pid.isTimeSampling();
  document.getElementById('outMin').value = pid.getOutputMin();
  document.getElementById('outMax').value = pid.getOutputMax();
  document.getElementById('icMode').value = pid.getIntegralCorrectionMode();
  document.getElementById('pMode').value = pid.getProportionalMode();
  document.getElementById('dMode').value = pid.getDerivativeMode();
}

/**
 * Apply PID parameters from UI controls
 */
function applyParameters() {
  const kp = parseFloat(document.getElementById('kp').value);
  const ki = parseFloat(document.getElementById('ki').value);
  const kd = parseFloat(document.getElementById('kd').value);
  const setpoint = parseFloat(document.getElementById('setpoint').value);
  const feedForward = parseFloat(document.getElementById('feedForward').value);
  const reverse = document.getElementById('reverse').checked;
  const timeSampling = document.getElementById('timeSampling').checked;
  const outMin = parseFloat(document.getElementById('outMin').value);
  const outMax = parseFloat(document.getElementById('outMax').value);
  const icMode = document.getElementById('icMode').value;
  const pMode = document.getElementById('pMode').value;
  const dMode = document.getElementById('dMode').value;

  pid.setKp(kp);
  pid.setKi(ki);
  pid.setKd(kd);
  pid.setSetpoint(setpoint);
  pid.setFeedForward(feedForward);
  pid.setReverse(reverse);
  pid.setTimeSampling(timeSampling);
  pid.setOutputLimits(outMin, outMax);
  pid.setIntegralCorrectionMode(icMode);
  pid.setProportionalMode(pMode);
  pid.setDerivativeMode(dMode);

  // Reset metrics when parameters change to get fresh measurements
  resetMetrics();

  console.log('PID parameters applied:', pid.toJSON());
}

/**
 * Reset simulation and PID state
 */
function resetSimulation() {
  pid.reset();
  grid = 0;
  load = 0;
  random_solar = 0;
  random_load_high = 0;
  random_load_low = 0;
  
  // Clear all charts
  charts.forEach(chart => {
    chart.data.labels = [];
    chart.data.datasets[0].data = [];
    chart.update('none');
  });

  // Reset metrics
  resetMetrics();

  console.log('Simulation reset');
}

/**
 * Pause simulation
 */
function pauseSimulation() {
  pid.pause();
  isPaused = true;
  document.getElementById('pause').disabled = true;
  document.getElementById('resume').disabled = false;
  console.log('Simulation paused');
}

/**
 * Resume simulation
 */
function resumeSimulation() {
  pid.resume();
  isPaused = false;
  document.getElementById('pause').disabled = false;
  document.getElementById('resume').disabled = true;
  console.log('Simulation resumed');
}

/**
 * Run one simulation step
 */
function simulationStep() {
  // Simulate random load variations (similar to C++ implementation)
  let delta = 0;

  // Generate some random noise between -20W and +20W about 30% of the time
  if (randomInt(0, 4) === 0) {
    if (random_load_low) {
      delta -= random_load_low;
      random_load_low = 0;
    } else {
      random_load_low = randomInt(-20, 20);
      delta += random_load_low;
    }
  }

  // Generate a random high load between 0W and 1000W
  if (randomInt(0, 49) === 0) {
    if (random_load_high) {
      delta -= random_load_high;
      random_load_high = 0;
    } else {
      random_load_high = randomInt(0, 1000);
      delta += random_load_high;
    }
  }

  // Simulate some solar production variation, generally going up over time
  if (randomInt(0, 4) === 0) {
    random_solar += randomInt(-50, 75);
    random_solar = constrain(random_solar, 0, 1000);
  }

  // Compute PID output
  const output = pid.compute(grid - random_solar);

  // If output <= 0, we have no power to divert to the load
  // If we have, we simulate a load that can only consume between 0 and 2000W
  const to_divert = output <= 0 ? 0 : Math.min(output, 2000);

  // Accumulate the load in the grid, considering its previous existing consumption
  grid -= load;
  grid += to_divert;
  load = to_divert;

  // We simulate the grid consumption by the load, applying some noise
  grid += delta;

  // Log state (similar to C++ Serial.printf)
  if (Math.random() < 0.05) { // Log occasionally to avoid console spam
    console.log(
      `Solar: ${random_solar.toFixed(2).padStart(7)} | ` +
      `Grid: ${pid.getInput().toFixed(2).padStart(7)} | ` +
      `pTerm: ${pid.getPTerm().toFixed(2).padStart(7)} | ` +
      `iTerm: ${pid.getITerm().toFixed(2).padStart(7)} | ` +
      `dTerm: ${pid.getDTerm().toFixed(2).padStart(7)} | ` +
      `Output: ${pid.getOutput().toFixed(2).padStart(7)} | ` +
      `Load: ${load.toFixed(2).padStart(7)} | ` +
      `Delta: ${delta.toFixed(2).padStart(7)}`
    );
  }

  // Update charts with data (order matches JSON payload: solar, grid, pTerm, iTerm, dTerm, output, load)
  const values = [
    random_solar,
    pid.getInput(),
    pid.getPTerm(),
    pid.getITerm(),
    pid.getDTerm(),
    pid.getOutput(),
    load
  ];

  updateCharts(values);
  
  // Update metrics
  updateMetrics(pid.getInput(), pid.getOutput(), pid.getSetpoint());
}

/**
 * Start simulation
 */
function startSimulation() {
  if (simulationInterval) {
    clearInterval(simulationInterval);
  }
  if (metricsData.updateInterval) {
    clearInterval(metricsData.updateInterval);
  }
  
  simulationInterval = setInterval(simulationStep, SEND_INTERVAL);
  
  // Update metrics display every second
  metricsData.updateInterval = setInterval(displayMetrics, 1000);
  
  console.log('Simulation started');
}

/**
 * Initialize everything on page load
 */
window.addEventListener('DOMContentLoaded', () => {
  // Create charts
  createCharts();

  // Initialize PID
  initializePID();

  // Bind button events
  document.getElementById('apply').addEventListener('click', applyParameters);
  document.getElementById('resetBtn').addEventListener('click', resetSimulation);
  document.getElementById('pause').addEventListener('click', pauseSimulation);
  document.getElementById('resume').addEventListener('click', resumeSimulation);

  // Start simulation
  startSimulation();

  console.log('PID Simulator ready');
});

// Cleanup on page unload
window.addEventListener('beforeunload', () => {
  if (simulationInterval) {
    clearInterval(simulationInterval);
  }
  if (metricsData.updateInterval) {
    clearInterval(metricsData.updateInterval);
  }
});
