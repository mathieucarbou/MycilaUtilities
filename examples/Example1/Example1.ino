#include <HardwareSerial.h>

#include <MycilaCircularBuffer.h>
#include <MycilaExpiringValue.h>
#include <MycilaPID.h>
#include <MycilaString.h>
#include <MycilaTime.h>

#define PID_COUNT 3

Mycila::CircularBuffer<int, 5> buffer;

Mycila::PID pids[PID_COUNT] = {
  Mycila::PID(),
  Mycila::PID(),
  Mycila::PID(),
};

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  buffer.add(1);
  buffer.add(2);
  buffer.add(3);
  buffer.add(4);
  buffer.add(5);
  buffer.add(6);
  buffer.add(7);

  buffer.dump(Serial);
  Serial.println(); // CircularBuffer(2,5/5)={6,7,3,4,5}

  Serial.println(Mycila::string::toLowerCase("Hello World").c_str()); // hello world
  Serial.println(Mycila::string::toUpperCase("Hello World").c_str()); // HELLO WORLD
  Serial.println(Mycila::string::trim("  Hello World  ").c_str());    // "Hello World"
  Serial.println(Mycila::string::startsWith("Hello World", "Hello")); // 1
  Serial.println(Mycila::string::endsWith("Hello World", "World"));   // 1
  Serial.println(Mycila::string::endsWith("Hello World", ""));        // 1
  Serial.println(Mycila::Time::toDHHMMSS(180245).c_str());            // 2d 02:04:05
  Serial.println(Mycila::Time::toMinutes("12:34"));                   // 754
  Serial.println(Mycila::Time::toMinutes("34"));                      // 34
  Serial.println(Mycila::Time::toMinutes(":34"));                     // 34
  Serial.println(Mycila::Time::toMinutes("34:"));                     // 2040
  Serial.println(Mycila::Time::toMinutes(":"));                       // 0
  Serial.println(Mycila::Time::toMinutes(""));                        // 0
  Serial.println(Mycila::Time::getUnixTime());                        // 0

  struct tm timeInfo;
  timeInfo.tm_hour = 12;
  timeInfo.tm_min = 34;
  Serial.println(Mycila::Time::timeInRange(timeInfo, "12:00", "13:00")); // 1
  Serial.println(Mycila::Time::timeInRange(timeInfo, "13:00", "12:00")); // 0

  Serial.println("Mycila::ExpiringValue");
  int i = 7;
  const int j = 8;
  Mycila::ExpiringValue<int> value(1000);
  value.update(10);
  Serial.println((bool)value);       // 1
  Serial.println(value.get());       // 10
  Serial.println(value.isPresent()); // 1
  Serial.println(value.orElse(0));   // 10
  Serial.println(value.orElse(i));   // 10
  Serial.println(value.orElse(j));   // 10
  delay(1200);
  Serial.println((bool)value);       // 0
  Serial.println(value.isPresent()); // 0
  Serial.println(value.orElse(20));  // 20
  Serial.println(value.orElse(i));   // 20
  Serial.println(value.orElse(j));   // 20

  for (size_t i = 0; i < PID_COUNT; i++) {
    pids[i].setOutputLimits(-300, 4000);
  }

  pids[0].setProportionalMode(Mycila::PID::ProportionalMode::P_ON_INPUT);
  pids[0].setDerivativeMode(Mycila::PID::DerivativeMode::D_ON_ERROR);
  pids[0].setIntegralCorrectionMode(Mycila::PID::IntegralCorrectionMode::IC_ADVANCED);
  pids[0].setKp(0);
  pids[0].setKi(0);
  pids[0].setKd(0.5);

  pids[1].setProportionalMode(Mycila::PID::ProportionalMode::P_ON_ERROR);
  pids[1].setDerivativeMode(Mycila::PID::DerivativeMode::D_ON_ERROR);
  pids[1].setIntegralCorrectionMode(Mycila::PID::IntegralCorrectionMode::IC_ADVANCED);
  pids[1].setKp(0);
  pids[1].setKi(0);
  pids[1].setKd(0.5);

  pids[2].setProportionalMode(Mycila::PID::ProportionalMode::P_ON_INPUT);
  pids[2].setDerivativeMode(Mycila::PID::DerivativeMode::D_ON_ERROR);
  pids[2].setIntegralCorrectionMode(Mycila::PID::IntegralCorrectionMode::IC_ADVANCED);
  pids[2].setKp(0.1);
  pids[2].setKi(0.2);
  pids[2].setKd(0.05);
}

// watts
float grid[PID_COUNT] = {-600, -600, -600};
float diverted[PID_COUNT] = {0, 0, 0};

void loop() {
  float rnd = random(0, 20) * (random(0, 2) == 0 ? -1 : 1);
  for (size_t i = 0; i < PID_COUNT; i++) {
    float output = pids[i].compute(grid[i]);
    Serial.printf("#%d: grid: %.2f, diverted: %.2f, output: %.2f, rnd: %.2f\n", i, grid[i], diverted[i], output, rnd);
    if (output > 0) {
      float adjust = output - diverted[i];
      grid[i] += adjust;
    }
    diverted[i] = output > 0 ? output : 0;
    grid[i] += rnd;
  }

  Serial.println();

  delay(300);
}
