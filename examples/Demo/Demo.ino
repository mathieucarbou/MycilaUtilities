// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2025 Mathieu Carbou
 */
#include <HardwareSerial.h>

#include <MycilaCircularBuffer.h>
#include <MycilaExpiringValue.h>
#include <MycilaString.h>
#include <MycilaTime.h>

#define PID_COUNT 3

Mycila::CircularBuffer<int, 5> buffer;

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
}

void loop() {
  delay(300);
}
