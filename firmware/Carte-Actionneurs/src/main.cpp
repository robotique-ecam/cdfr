#include <Arduino.h>
#include <Wire.h>

#define ADDRESS 0x40

void receiveEvent(int n) {
  if (n == 1)
    PORTD = Wire.read();
}

void setup() {
  PORTD = 0x00;
  DDRD = 0xFF;
  Wire.begin(ADDRESS);
  Wire.onReceive(receiveEvent);
}

void loop() {}
