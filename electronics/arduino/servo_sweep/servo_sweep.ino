// Smooth back-and-forth sweep on one servo (first servo test: base servo on pin 3).
#include <Servo.h>
Servo s;
void setup() { s.attach(3); }
void loop() {
  float t = millis() / 1000.0;
  s.write(90 + 60 * sin(t));   // eased sweep between 30 and 150 degrees
  delay(15);
}
