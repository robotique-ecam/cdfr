#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <TinyWireS.h>
#include "speedramp.hpp"

#define OBELIX

#ifdef ASTERIX
// #define I2C_ADDR 0x10
// #define INVERT 1
#define I2C_ADDR 0x11
#define INVERT 0
#endif

#ifdef OBELIX
// #define I2C_ADDR 0x10
// #define INVERT 0
#define I2C_ADDR 0x11
#define INVERT 1
#endif

#define PIN_DIR 4
#define PIN_ENA 1
#define PIN_STEP 3

#define abs(N) ((N < 0) ? (-N) : (N))
#define sign(N) ((N < 0) ? (1) : (0))

uint16_t old_steps = 0;
volatile uint16_t steps = 0;

void setup() {
  /* Setup IO */
  PORTB = 0;
  DDRB |= (1 << PIN_DIR) | (1 << PIN_STEP) | (1 << PIN_ENA);
  PORTB |= (1 << PIN_ENA);

  /* setup timer 1 */
  TIMSK |= (1 << OCIE1A);
}

void step() {
  PORTB |= (1 << PIN_STEP);
  steps++;
  _delay_us(4);
  PORTB &= ~(1 << PIN_STEP);
}

ISR(TIMER1_COMPA_vect) {
  step();
  TCNT1 = 0;
}

void on_receive_command(uint8_t n) {
  int8_t data = TinyWireS.receive();
  if (data != -128) {
    TCNT1 = 0;
    old_steps = steps;
    steps = 0;
    /* Send direction according to data sign */
    PORTB ^= (-(sign(data) ^ INVERT) ^ PORTB) & (1 << PIN_DIR);
    uint8_t data_index = abs(data);
    OCR1A = comparator[data_index];
    TCCR1 = (TCCR1 & 0xf0) | prescaler[data_index];
    TinyWireS.send((uint8_t)old_steps);
    TinyWireS.send(old_steps >> 8);
  } else {
    /* Handle magic command */
    uint8_t cmd = TinyWireS.receive();
    if (cmd == 0x10) {
      DDRB |= (1 << PIN_ENA);
      PORTB |= (1 << PIN_ENA);
    } else if (cmd == 0x11) {
      DDRB &= ~(1 << PIN_ENA);
      PORTB &= ~(1 << PIN_ENA);
    }
  }
}

int main(int argc, char const *argv[]) {

  setup();

  TinyWireS.begin(I2C_ADDR);
  TinyWireS.onReceive(on_receive_command);

  sei();

  while (1) {
    TinyWireS_stop_check();
  }
  return 0;
}
