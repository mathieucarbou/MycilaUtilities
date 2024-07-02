#include <Arduino.h>

#include <MycilaCircularBuffer.h>
#include <MycilaString.h>
#include <MycilaTime.h>
#include <MycilaPID.h>

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

  Serial.println(Mycila::Str::lowerCaseCopy("Hello World"));
  Serial.println(Mycila::Time::toDHHMMSS(180245));
  Serial.println(Mycila::Time::toMinutes("12:34"));
  Serial.println(Mycila::Time::getUnixTime());
}

// Destroy default Arduino async task
void loop() { vTaskDelete(NULL); }
