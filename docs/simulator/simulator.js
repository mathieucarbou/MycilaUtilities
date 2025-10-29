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
  pid.setDerivativeMode(pid.DerivativeMode.ON_INPUT);
  pid.setKp(0.1);
  pid.setKi(0.2);
  pid.setKd(0.05);
  pid.setSetpoint(0);
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
}

/**
 * Start simulation
 */
function startSimulation() {
  if (simulationInterval) {
    clearInterval(simulationInterval);
  }
  simulationInterval = setInterval(simulationStep, SEND_INTERVAL);
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
});
