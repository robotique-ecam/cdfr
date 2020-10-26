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
#define I2C_ADDR 0x10
#define INVERT 0
// #define I2C_ADDR 0x11
// #define INVERT 1
#endif

#define PIN_DIR 4
#define PIN_ENA 1
#define PIN_STEP 3

#define abs(N) ((N < 0) ? (-N) : (N))
#define sign(N) ((N < 0) ? (1) : (0))

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 4 )
#endif

volatile uint16_t steps = 0;
volatile uint16_t old_steps = 0;
volatile uint8_t prescaler_value = 0;

void setup() {
  /* Setup IO */
  PORTB = 0;
  DDRB |= (1 << PIN_DIR) | (1 << PIN_STEP) | (1 << PIN_ENA);
  PORTB |= (1 << PIN_ENA);

  /* setup timer 1 */
  TIMSK |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect) {
  /* Disable comparator A interrupts */
  TIMSK &= ~(1 << OCIE1A);
  /* Start the pulse */
  PORTB |= (1 << PIN_STEP);
  steps++;
  /* Reset counter */
  TCNT1 = 0;
  /* Delay for x us by dividing clock by 16 (1us per count) and counting */
  TCCR1 = (TCCR1 & 0xf0) | 5;
  OCR1B = 10;
  TIMSK |= (1 << OCIE1B);
}

ISR(TIMER1_COMPB_vect) {
  /* Disable comparator B interrupts */
  TIMSK &= ~(1 << OCIE1B);
  /* Reset counter */
  TCNT1 = 0;
  /* Restore values */
  TCCR1 = (TCCR1 & 0xf0) | prescaler_value;
  /* Stop the pulse */
  PORTB &= ~(1 << PIN_STEP);
  /* Enable compare interrupt */
  TIMSK |= (1 << OCIE1A);
}

/* On I2C read request */
void onRequest() {
  TinyWireS.send((uint8_t)old_steps);
  TinyWireS.send(old_steps >> 8);
}

/* On I2C write request */
void onReceive(uint8_t n) {
  if ((n > 0) | (n < TWI_RX_BUFFER_SIZE)) {
    int8_t data = TinyWireS.receive();
    if (data != -128) {
      TCNT1 = 0;
      old_steps = steps;
      steps = 0;
      /* Send direction according to data sign */
      PORTB ^= (-(sign(data) ^ INVERT) ^ PORTB) & (1 << PIN_DIR);
      uint8_t data_index = abs(data);
      prescaler_value = prescaler[data_index];
      OCR1A = comparator[data_index];
      TCCR1 = (TCCR1 & 0xF0) | prescaler_value;
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
}

int main(int argc, char const *argv[]) {

  setup();

  TinyWireS.begin(I2C_ADDR);
  TinyWireS.onReceive(onReceive);
  TinyWireS.onRequest(onRequest);

  sei();

  while (1) {
    TinyWireS_stop_check();
  }
  return 0;
}
