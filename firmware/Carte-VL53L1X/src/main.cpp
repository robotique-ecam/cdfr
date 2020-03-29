#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <LowPower.h>


VL53L1X sensors[8];
uint8_t addr = 0x30;


void setup() {
  // All sensors pins are outputs and set to LOW
  PORTD = 0x00;
  DDRD = 0x00;

  // Init I2C
  Wire.begin();
  Wire.setClock(400000);

  for (uint8_t i = 0; i < 8; i++ ) {
    // Enable sensor
    PORTD += (1 << i);
    // Startup delay for sensor
    delay(200);
    // Init sensor and set I2C address
    sensors[i].init();
    sensors[i].setAddress(addr);

    // Sensor params
    sensors[i].setDistanceMode(VL53L1X::Long);
    sensors[i].setMeasurementTimingBudget(50000);
    sensors[i].startContinuous(50);
    sensors[i].setTimeout(100);

    addr++;
  }

  LowPower.idle(SLEEP_FOREVER, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
}


void loop() {

}
