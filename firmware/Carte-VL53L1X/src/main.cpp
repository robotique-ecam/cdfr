#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <LowPower.h>

VL53L1X sensors[8];
uint8_t addr = 0x30;

void setup() {
  // All sensors XSHUT pins are outputs and set to HIGH
  DDRD = 0xFF;
  PORTD = 0x00;

  // Init I2C
  Wire.begin();
  Wire.setClock(400000);

  for (uint8_t i = 0; i < 8; i++) {
    // Enable sensor i
    PORTD |= (1 << i);

    // Startup delay for sensor
    delay(100);

    // Check if VL53L1X is connected and initialize it
    // Init sensor and set I2C address
    Wire.beginTransmission(0x29);
    if (Wire.endTransmission() == 0) {
      sensors[i].init();
      sensors[i].setAddress(addr);
    }

    addr++;
  }

  LowPower.idle(SLEEP_FOREVER, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
}

void loop() {}
