#include <Arduino.h>
#include <Wire.h>

#define ADDRESS 0x40

void receiveEvent(int n) {
  // Call upon I2C write byte from master device
  if (n == 1)
    PORTD = Wire.read();
}

void requestEvent() {
  // Call upon I2C read byte from master device
  Wire.write(PORTD);
}

void setup() {
  PORTD = 0x00;
  DDRD = 0xFF;
  Wire.begin(ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {}
