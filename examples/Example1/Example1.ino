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
  Serial.println();

  Serial.println(Mycila::string::toLowerCase("Hello World").c_str());
  Serial.println(Mycila::string::toUpperCase("Hello World").c_str());
  Serial.println(Mycila::string::trim("  Hello World  ").c_str());
  Serial.println(Mycila::string::startsWith("Hello World", "Hello"));
  Serial.println(Mycila::string::endsWith("Hello World", "World"));
  Serial.println(Mycila::string::endsWith("Hello World", ""));
  Serial.println(Mycila::Time::toDHHMMSS(180245).c_str());
  Serial.println(Mycila::Time::toMinutes("12:34")); // 754
  Serial.println(Mycila::Time::toMinutes("34"));    // 34
  Serial.println(Mycila::Time::toMinutes(":34"));   // 34
  Serial.println(Mycila::Time::toMinutes("34:"));   // 2040
  Serial.println(Mycila::Time::toMinutes(":"));     // 0
  Serial.println(Mycila::Time::toMinutes(""));      // 0
  Serial.println(Mycila::Time::getUnixTime());

  struct tm timeInfo;
  timeInfo.tm_hour = 12;
  timeInfo.tm_min = 34;
  Serial.println(Mycila::Time::timeInRange(timeInfo, "12:00", "13:00"));
  Serial.println(Mycila::Time::timeInRange(timeInfo, "13:00", "12:00"));

  Serial.println("Mycila::ExpiringValue");
  int i = 7;
  const int j = 8;
  Mycila::ExpiringValue<int> value(1000);
  value.update(10);
  Serial.println((bool)value);
  Serial.println(value.get());
  Serial.println(value.isPresent());
  Serial.println(value.orElse(0));
  Serial.println(value.orElse(i));
  Serial.println(value.orElse(j));
  delay(1200);
  Serial.println((bool)value);
  Serial.println(value.isPresent());
  Serial.println(value.orElse(20));
  Serial.println(value.orElse(i));
  Serial.println(value.orElse(j));
}

// Destroy default Arduino async task
void loop() { vTaskDelete(NULL); }
