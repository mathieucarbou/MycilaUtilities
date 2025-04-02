#include <HardwareSerial.h>

#include <MycilaCircularBuffer.h>
#include <MycilaExpiringValue.h>
#include <MycilaPID.h>
#include <MycilaString.h>
#include <MycilaTime.h>

Mycila::CircularBuffer<int, 5> buffer;
Mycila::PID pid;

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

  pid.setOutputLimits(-500, 5000);

  // pid.setProportionalMode(Mycila::PID::ProportionalMode::P_ON_INPUT);
  // pid.setDerivativeMode(Mycila::PID::DerivativeMode::D_ON_ERROR);
  // pid.setIntegralCorrectionMode(Mycila::PID::IntegralCorrectionMode::IC_ADVANCED);
  // pid.setKp(0.1);
  // pid.setKi(0.2);
  // pid.setKd(0.05);

  pid.setProportionalMode(Mycila::PID::ProportionalMode::P_ON_ERROR);
  pid.setDerivativeMode(Mycila::PID::DerivativeMode::D_ON_INPUT);
  pid.setIntegralCorrectionMode(Mycila::PID::IntegralCorrectionMode::IC_CLAMP);
  pid.setKp(0);
  pid.setKi(1);
  pid.setKd(0);

  // pid.setProportionalMode(Mycila::PID::ProportionalMode::P_ON_INPUT);
  // pid.setDerivativeMode(Mycila::PID::DerivativeMode::D_ON_ERROR);
  // pid.setIntegralCorrectionMode(Mycila::PID::IntegralCorrectionMode::IC_ADVANCED);
  // pid.setKp(0);
  // pid.setKi(1);
  // pid.setKd(0);

  // pid.setProportionalMode(Mycila::PID::ProportionalMode::P_ON_INPUT);
  // pid.setDerivativeMode(Mycila::PID::DerivativeMode::D_ON_ERROR_RATE);
  // pid.setIntegralCorrectionMode(Mycila::PID::IntegralCorrectionMode::IC_ADVANCED);
  // pid.setKp(0);
  // pid.setKi(0);
  // pid.setKd(1);
}

float grid = -600; // in Watts
float diverted = 0;

void loop() {
  float output = pid.compute(grid);
  float rnd = random(0, 20) * (random(0, 2) == 0 ? -1 : 1);
  Serial.printf("grid: %.2f, diverted: %.2f, output: %.2f, rnd: %.2f\n", grid, diverted, output, rnd);
  if (output > 0) {
    float adjust = output - diverted;
    grid += adjust;
  }
  diverted = output > 0 ? output : 0;
  grid += rnd;
  delay(300);
}
