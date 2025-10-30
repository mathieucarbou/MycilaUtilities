// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2025 Mathieu Carbou
 */
#include <Arduino.h>

#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <MycilaPID.h>
#include <WiFi.h>

static AsyncWebServer server(80);
static AsyncWebSocketMessageHandler wsHandler;
static AsyncWebSocket ws("/ws", wsHandler.eventHandler());
static Mycila::PID pid;

static float grid = 0;             // grid power in watts
static float load = 0;             // load where to divert excess power watts
static float random_load_high = 0; // random high load up to 1000W
static float random_load_low = 0;  // random low load between -20W and +20W
static float random_solar = 0;     // random solar power generation between 0W and 1000W

uint32_t lastSend = 0;
const uint32_t sendInterval = 330; // ms to simulate a JSY response rate

// HTML content generated with Copilot's help and based on C++ implementation below
String page = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>Web PID</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    body{font-family:Arial;margin:12px}
    #chart{width:100%;height:240px}
    .controls{display:flex;gap:8px;flex-wrap:wrap;margin-top:8px}
    label{display:flex;flex-direction:column;font-size:12px}
  </style>
</head>
<body>
  <h2>PID Simulator</h2>
  <div id="charts"></div>
  <div class="controls">
    <label>Kp<input id="kp" type="number" step="0.01" value="0.2"></label>
    <label>Ki<input id="ki" type="number" step="0.01" value="0.1"></label>
    <label>Kd<input id="kd" type="number" step="0.01" value="0.05"></label>
    <label>Setpoint<input id="setpoint" type="number" step="1" value="-600"></label>
    <label>Feed Forward<input id="feedForward" type="number" step="1" value="0"></label>
    <label>Reverse<input id="reverse" type="checkbox"></label>
    <label>TimeSampling<input id="timeSampling" type="checkbox"></label>
    <label>Output Min<input id="outMin" type="number" step="1" value="-300"></label>
    <label>Output Max<input id="outMax" type="number" step="1" value="3000"></label>
    <label>IC Mode<select id="icMode"><option value="clamp">clamp</option><option value="off">off</option></select></label>
    <label>P Mode<select id="pMode"><option value="error">error</option><option value="input">input</option></select></label>
    <label>D Mode<select id="dMode"><option value="error">error</option><option value="input">input</option></select></label>
    
  <button id="apply">Apply</button>
  <button id="resetBtn">Reset</button>
  <button id="pause">Pause</button>
  <button id="resume">Resume</button>
  </div>

  <script>
  const MAX_POINTS = 100;
  const ws = new WebSocket('ws://' + location.host + '/ws');
  // Match the JSON payload order and keys emitted by the server
  // Server sends: solar, grid (pid input), pTerm, iTerm, dTerm, output, load
  const CHART_KEYS = ['Solar (W)','Grid (W)','pTerm (W)','iTerm (W)','dTerm (W)','Output (W)','Load (W)'];
  const CHART_COLORS = ['#9A6324','#e6194b','#3cb44b','#ffe119','#4363d8','#f58231','#911eb4'];
  let charts = [];
  let dataBuf = [];

    function createChart() {
      const container = document.getElementById('charts');
      container.innerHTML = '';
      charts = [];
      for (let i = 0; i < CHART_KEYS.length; i++) {
        // Create a wrapper with a clickable label to show/hide the chart
        const wrapper = document.createElement('div');
        wrapper.style.marginBottom = '12px';

        const title = document.createElement('div');
        title.textContent = CHART_KEYS[i];
        title.style.cursor = 'pointer';
        title.style.fontSize = '14px';
        title.style.fontWeight = '600';
        title.style.marginBottom = '6px';
        title.id = 'chartTitle' + i;
        // toggle chart visibility when clicking the title
        title.onclick = () => {
          const c = document.getElementById('chart' + i);
          if (!c) return;
          c.style.display = c.style.display === 'none' ? '' : 'none';
        };

        const canvas = document.createElement('canvas');
        canvas.id = 'chart' + i;
        canvas.style.width = '100%';
        canvas.style.height = '120px';
        wrapper.appendChild(title);
        wrapper.appendChild(canvas);
        container.appendChild(wrapper);

        const ctx = canvas.getContext('2d');
        const cfg = {
          type: 'line',
          data: { labels: [], datasets: [{ label: CHART_KEYS[i], data: [], borderColor: CHART_COLORS[i], fill: false, showLine: true, spanGaps: true }] },
          options: {
            animation: false,
            plugins: { legend: { display: false } },
            elements: { line: { tension: 0.4, cubicInterpolationMode: 'monotone' } },
            scales: { x: { display: false } }
          }
        };
        charts.push(new Chart(ctx, cfg));
      }
    }

    ws.onopen = () => console.log('ws open');
    ws.onmessage = (ev) => {
      try {
        const msg = JSON.parse(ev.data);
        if (msg.type === 'sample') {
          // server JSON payload: solar, grid (input), pTerm, iTerm, dTerm, output, load
          const values = [msg.solar, msg.grid, msg.pTerm, msg.iTerm, msg.dTerm, msg.output, msg.load];
          for (let i = 0; i < values.length; i++) {
            const c = charts[i];
            c.data.labels.push('');
            c.data.datasets[0].data.push(values[i]);
            while (c.data.labels.length > MAX_POINTS) {
              c.data.labels.shift();
              c.data.datasets[0].data.shift();
            }
            c.update();
          }
        } else if (msg.type === 'pid') {
          // server sent PID state (on connect) -> populate controls
          try {
            if (typeof msg.setpoint !== 'undefined') document.getElementById('setpoint').value = msg.setpoint;
            if (typeof msg.feedForward !== 'undefined') document.getElementById('feedForward').value = msg.feedForward;
            if (typeof msg.reverse !== 'undefined') document.getElementById('reverse').checked = !!msg.reverse;
            if (typeof msg.timeSampling !== 'undefined') document.getElementById('timeSampling').checked = !!msg.timeSampling;
            if (typeof msg.output_min !== 'undefined') document.getElementById('outMin').value = msg.output_min;
            if (typeof msg.output_max !== 'undefined') document.getElementById('outMax').value = msg.output_max;
            if (typeof msg.icMode !== 'undefined') document.getElementById('icMode').value = msg.icMode;
            if (typeof msg.pMode !== 'undefined') document.getElementById('pMode').value = msg.pMode;
            if (typeof msg.dMode !== 'undefined') document.getElementById('dMode').value = msg.dMode;
            // set reset field to setpoint by default
            if (typeof msg.setpoint !== 'undefined') document.getElementById('reset').value = msg.setpoint;

            // enable/disable pause/resume buttons according to enabled
            if (typeof msg.enabled !== 'undefined') {
              const paused = !msg.enabled;
              document.getElementById('pause').disabled = paused;
              document.getElementById('resume').disabled = !paused;
            }
          } catch(e) { console.warn('failed populating controls', e); }
        }
      } catch(e){console.error(e)}
    };

    document.getElementById('apply').onclick = () => {
      const payload = {
        type: 'params',
        kp: parseFloat(document.getElementById('kp').value),
        ki: parseFloat(document.getElementById('ki').value),
        kd: parseFloat(document.getElementById('kd').value),
        setpoint: parseFloat(document.getElementById('setpoint').value),
        feedForward: parseFloat(document.getElementById('feedForward').value),
        reverse: document.getElementById('reverse').checked,
        timeSampling: document.getElementById('timeSampling').checked,
        output_min: parseFloat(document.getElementById('outMin').value),
        output_max: parseFloat(document.getElementById('outMax').value),
        icMode: document.getElementById('icMode').value,
        pMode: document.getElementById('pMode').value,
        dMode: document.getElementById('dMode').value
      };
      ws.send(JSON.stringify(payload));
    };

    document.getElementById('pause').onclick = () => {
      ws.send(JSON.stringify({type:'cmd', cmd:'pause'}));
    };

    document.getElementById('resume').onclick = () => {
      ws.send(JSON.stringify({type:'cmd', cmd:'resume'}));
    };

    document.getElementById('resetBtn').onclick = () => {
      // send reset command using current setpoint
      const sp = parseFloat(document.getElementById('setpoint').value);
      ws.send(JSON.stringify({type:'cmd', cmd:'reset', reset: sp}));
    };


    window.onload = createChart;
  </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(1);

  WiFi.softAP("esp-captive");

  pid.setReverse(false);
  pid.setTimeSampling(false);
  pid.setOutputLimits(-300, 4000);
  pid.setIntegralCorrectionMode(Mycila::PID::IntegralCorrectionMode::CLAMP);
  pid.setProportionalMode(Mycila::PID::ProportionalMode::ON_INPUT);
  pid.setDerivativeMode(Mycila::PID::DerivativeMode::ON_ERROR);
  pid.setKp(0.1);
  pid.setKi(0.4);
  pid.setKd(0.05);

  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/html", page);
  });

  wsHandler.onMessage([](AsyncWebSocket* server, AsyncWebSocketClient* client, const uint8_t* data, size_t len) {
    // Parse JSON using ArduinoJson
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, data, len);
    if (err) {
      Serial.printf("JSON parse error: %s\n", err.c_str());
    } else {
      // Accept either a wrapper {"params": {...}} or a flat object with keys
      JsonObject obj = doc.as<JsonObject>();
      if (!obj["params"].isNull())
        obj = obj["params"].as<JsonObject>();

      // parameters
      if (!obj["kp"].isNull()) {
        pid.setKp(obj["kp"]);
        Serial.printf("Kp updated to %.3f\n", pid.getKp());
      }

      if (!obj["ki"].isNull()) {
        pid.setKi(obj["ki"]);
        Serial.printf("Ki updated to %.3f\n", pid.getKi());
      }

      if (!obj["kd"].isNull()) {
        pid.setKd(obj["kd"]);
        Serial.printf("Kd updated to %.3f\n", pid.getKd());
      }

      if (!obj["feedForward"].isNull()) {
        pid.setFeedForward(obj["feedForward"]);
        Serial.printf("FeedForward updated to %.2f\n", pid.getFeedForward());
      }

      if (!obj["timeSampling"].isNull()) {
        pid.setTimeSampling(obj["timeSampling"]);
        Serial.printf("TimeSampling mode updated to %d\n", pid.isTimeSampling());
      }

      if (!obj["setpoint"].isNull()) {
        pid.setSetpoint(obj["setpoint"]);
        Serial.printf("Setpoint updated to %.2f\n", pid.getSetpoint());
      }

      if (!obj["reverse"].isNull()) {
        pid.setReverse(obj["reverse"]);
        Serial.printf("Reverse mode updated to %d\n", pid.isReverse());
      }

      if (!obj["output_min"].isNull() || !obj["output_max"].isNull()) {
        float minv = !obj["output_min"].isNull() ? obj["output_min"].as<float>() : pid.getOutputMin();
        float maxv = !obj["output_max"].isNull() ? obj["output_max"].as<float>() : pid.getOutputMax();
        pid.setOutputLimits(minv, maxv);
        Serial.printf("Output limits updated to [%.1f..%.1f]\n", pid.getOutputMin(), pid.getOutputMax());
      }

      if (!obj["pMode"].isNull()) {
        pid.setProportionalMode(obj["pMode"].as<String>() == "input" ? Mycila::PID::ProportionalMode::ON_INPUT : Mycila::PID::ProportionalMode::ON_ERROR);
        Serial.printf("Proportional mode updated to %s\n", pid.getProportionalMode() == Mycila::PID::ProportionalMode::ON_ERROR ? "error" : "input");
      }

      if (!obj["dMode"].isNull()) {
        pid.setDerivativeMode(obj["dMode"].as<String>() == "input" ? Mycila::PID::DerivativeMode::ON_INPUT : Mycila::PID::DerivativeMode::ON_ERROR);
        Serial.printf("Derivative mode updated to %s\n", pid.getDerivativeMode() == Mycila::PID::DerivativeMode::ON_ERROR ? "error" : "input");
      }

      if (!obj["icMode"].isNull()) {
        pid.setIntegralCorrectionMode(obj["icMode"].as<String>() == "off" ? Mycila::PID::IntegralCorrectionMode::OFF : Mycila::PID::IntegralCorrectionMode::CLAMP);
        Serial.printf("Integral Correction mode updated to %s\n", pid.getIntegralCorrectionMode() == Mycila::PID::IntegralCorrectionMode::OFF ? "off" : "clamp");
      }

      // commands
      if (obj["type"].as<String>() == "cmd") {
        String cmd = obj["cmd"].as<String>();
        if (cmd == "pause") {
          pid.pause();
          Serial.println("PID paused");
        } else if (cmd == "resume") {
          pid.resume();
          Serial.println("PID resumed");
        } else if (cmd == "reset") {
          pid.reset();
          grid = 0;
          load = 0;
          random_solar = 0;
          random_load_high = 0;
          random_load_low = 0;
          Serial.println("System reset");
        }
      }
    }
  });

  wsHandler.onConnect([](AsyncWebSocket* server, AsyncWebSocketClient* client) {
    JsonDocument doc;
    JsonObject obj = doc.to<JsonObject>();
    obj["type"] = "pid";
    obj["enabled"] = pid.isEnabled();
    obj["reverse"] = pid.isReverse();
    obj["timeSampling"] = pid.isTimeSampling();
    obj["pMode"] = pid.getProportionalMode() == Mycila::PID::ProportionalMode::ON_ERROR ? "error" : "input";
    obj["dMode"] = pid.getDerivativeMode() == Mycila::PID::DerivativeMode::ON_ERROR ? "error" : "input";
    obj["icMode"] = pid.getIntegralCorrectionMode() == Mycila::PID::IntegralCorrectionMode::OFF ? "off" : "clamp";
    obj["output_min"] = pid.getOutputMin();
    obj["output_max"] = pid.getOutputMax();
    obj["setpoint"] = pid.getSetpoint();
    obj["feedForward"] = pid.getFeedForward();
    AsyncWebSocketMessageBuffer* buffer = ws.makeBuffer(measureJson(doc));
    serializeJson(doc, buffer->get(), buffer->length());
    client->text(buffer);
  });

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  if (millis() - lastSend >= sendInterval) {
    // simulate one step similarly to PIDSim
    float delta = 0;

    // generate some random noise between -20W and +20W about 30% of the time
    if (random(0, 5) == 0) {
      if (random_load_low) {
        delta -= random_load_low;
        random_load_low = 0;
      } else {
        random_load_low = random(-20, 20);
        delta += random_load_low;
      }
    }

    // generate a random high load between 0W and 1000W
    if (random(0, 50) == 0) {
      if (random_load_high) {
        delta -= random_load_high;
        random_load_high = 0;
      } else {
        random_load_high = random(0, 1000);
        delta += random_load_high;
      }
    }

    // simulate some solar production variation, generally going up over time
    if (random(0, 5) == 0) {
      random_solar += random(-50, 75);
      random_solar = constrain(random_solar, 0, 1000);
    }

    const float output = pid.compute(grid - random_solar);

    // if output <= 0, we have no power to divert to the load
    // if we have, we simulate a load that can only consume between 0 and 1000W
    const float to_divert = output <= 0 ? 0 : (output > 2000 ? 2000 : output);

    // accumulate the load in the grid, considering its previous existing consumption
    grid -= load;
    grid += to_divert;
    load = to_divert;

    // we simulate the grid consumption by the load, applying some noise
    grid += delta;

    Serial.printf("Solar: %7.2f | Grid: %7.2f | pTerm: %7.2f | iTerm: %7.2f | dTerm: %7.2f | Output: %7.2f | Diverted: %7.2f | Delta: %7.2f\n",
                  random_solar,
                  pid.getInput(),
                  pid.getPTerm(),
                  pid.getITerm(),
                  pid.getDTerm(),
                  pid.getOutput(),
                  load,
                  delta);

    // build JSON sample with PID state
    JsonDocument doc;
    JsonObject obj = doc.to<JsonObject>();
    obj["type"] = "sample";
    obj["solar"] = random_solar;
    obj["grid"] = pid.getInput();
    obj["pTerm"] = pid.getPTerm();
    obj["iTerm"] = pid.getITerm();
    obj["dTerm"] = pid.getDTerm();
    obj["output"] = pid.getOutput();
    obj["load"] = load;

    AsyncWebSocketMessageBuffer* buffer = ws.makeBuffer(measureJson(doc));
    serializeJson(doc, buffer->get(), buffer->length());
    ws.textAll(buffer);

    lastSend = millis();
  }
}
