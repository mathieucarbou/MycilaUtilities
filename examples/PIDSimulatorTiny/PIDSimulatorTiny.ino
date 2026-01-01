// SPDX-License-Identifier: MIT
/*
 * Copyright (C) Mathieu Carbou
 */
#include <Arduino.h>
#include <HardwareSerial.h>
#include <MycilaCircularBuffer.h>
#include <MycilaExpiringValue.h>
#include <MycilaPID.h>
#include <MycilaString.h>
#include <MycilaTime.h>

#define PID_COUNT 4
#define ROUNDS    100

static Mycila::PID pids[PID_COUNT] = {
  Mycila::PID(),
  Mycila::PID(),
  Mycila::PID(),
  Mycila::PID(),
};

// watts
static float grid[PID_COUNT] = {0, 0, 0, 0};
static float loads[PID_COUNT] = {0, 0, 0, 0};
static float random_load_high = 0;
static float random_load_low = 0;
static float random_solar = 0;

uint32_t lastSend = 0;
const uint32_t sendInterval = 330; // ms to simulate a JSY response rate

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  for (size_t i = 0; i < PID_COUNT; i++) {
    pids[i].setReverse(false);
    pids[i].setTimeSampling(false);
    pids[i].setOutputLimits(-300, 3000);
    pids[i].setIntegralCorrectionMode(Mycila::PID::IntegralCorrectionMode::CLAMP);
  }

  pids[0].setProportionalMode(Mycila::PID::ProportionalMode::ON_ERROR);
  pids[0].setKp(0.2);
  pids[0].setKi(0.1);
  pids[0].setKd(0.05);

  pids[1].setProportionalMode(Mycila::PID::ProportionalMode::ON_INPUT);
  pids[1].setKp(0.1);
  pids[1].setKi(0.2);
  pids[1].setKd(0.05);

  pids[2].setProportionalMode(Mycila::PID::ProportionalMode::ON_ERROR);
  pids[2].setKp(0.2);
  pids[2].setKi(0.1);
  pids[2].setKd(0.05);

  pids[3].setProportionalMode(Mycila::PID::ProportionalMode::ON_INPUT);
  pids[3].setKp(0.1);
  pids[3].setKi(0.2);
  pids[3].setKd(0.05);
}

void loop() {
  if (millis() - lastSend >= sendInterval) {
    float delta = 0;

    // generate a random low load between -20W and +20W about 30% of the time
    if (random(0, 5) == 0) {
      if (random_load_low) {
        delta -= random_load_low;
        random_load_low = 0;
      } else {
        random_load_low = random(-20, 20);
        delta += random_load_low;
      }
    }

    // generate a random high load about 3 times
    if (random(0, 50) == 0) {
      if (random_load_high) {
        delta -= random_load_high;
        random_load_high = 0;
      } else {
        random_load_high = random(0, 1000);
        delta += random_load_high;
      }
    }

    // simulate some solar production variation
    if (random(0, 5) == 0) {
      random_solar += random(-50, 75);
      random_solar = constrain(random_solar, 0, 1000);
    }

    for (size_t i = 0; i < PID_COUNT; i++) {
      // pid call
      float output = pids[i].compute(grid[i] - random_solar);

      // if output <= 0, we have no power to divert to the load
      // if we have, we simulate a load that can only consume between 0 and 1000W
      const float to_divert = output <= 0 ? 0 : (output > 2000 ? 2000 : output);

      // accumulate the load in the grid, considering its previous existing consumption
      grid[i] -= loads[i];
      grid[i] += to_divert;
      loads[i] = to_divert;

      // we simulate the grid consumption by the load, applying some noise
      grid[i] += delta;

      // display
      Serial.printf("[%d] ", i + 1);
      Serial.printf("Solar: %7.2f | Grid: %7.2f | pTerm: %7.2f | iTerm: %7.2f | dTerm: %7.2f | Output: %7.2f | Diverted: %7.2f | Delta: %7.2f\n",
                    pids[i].getInput(),
                    pids[i].getPTerm(),
                    pids[i].getITerm(),
                    pids[i].getDTerm(),
                    pids[i].getOutput(),
                    loads[i],
                    delta);
    }

    Serial.println();

    lastSend = millis();
  }
}
