#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <TinyWireS.h>
#include "speedramp.hpp"

#define STEP 1
#define I2C_ADDR 0x12
#define STEPS_PER_MM 28 // Microstepping included

#define PIN_DIR 4
#define PIN_ENA 1
#define PIN_STEP 3

#define abs(N) ((N < 0) ? (-N) : (N))
#define sign(N) ((N < 0) ? (1) : (0))

bool move_up = true;
uint32_t steps_goal = 0;
uint32_t steps_position = 0;
uint32_t max_speed_steps = 0;

volatile uint8_t i = 0;
volatile uint8_t index = 1;
volatile uint16_t s = 0;
volatile bool raising = false;
volatile bool max = false;
volatile bool falling = false;

void setup() {
  /* Setup IO */
  PORTB = 0;
  DDRB |= (1 << PIN_DIR) | (1 << PIN_STEP) | (1 << PIN_ENA);

  /* setup timer 1 */
  TIMSK |= (1 << OCIE1A);
}

void step() {
  PORTB |= (1 << PIN_STEP);
  _delay_us(2);
  PORTB &= ~(1 << PIN_STEP);
}

ISR(TIMER1_COMPA_vect) {
  if (raising) { // Going up to max speed
    i++;
    // After STEP steps
    if (i >= STEP) {
      i = 0;
      index++;
      // If we acheived max speed
      if (index > ramp_size - 1) {
        max = true;
        raising = false;
        index = ramp_size - 1;
      }
      // Apply setting
      OCR1A = comparator[index];
      TCCR1 = (TCCR1 & 0xF0) | prescaler[index];
    }
  } else if (max) { // reached max speed
    s++;
    // If completed max speed
    if (s >= max_speed_steps) {
      falling = true;
      max = false;
      i, s = 0;
      index = ramp_size;
    }
  } else if (falling) { // falling
    i++;
    // After STEP steps
    if (i >= STEP) {
      i = 0;
      index--;
      // If we acheived speed
      if (index == 0) {
        raising = true;
        falling = false;
      }
      // Apply setting
      OCR1A = comparator[index];
      TCCR1 = (TCCR1 & 0xF0) | prescaler[index];
    }
  }
  step();
  TCNT1 = 0;
}

void on_receive_command(uint8_t n) {
  // Sanity check
  if (n == 1) {
    steps_goal = TinyWireS.receive();
    steps_goal *= STEPS_PER_MM;
    uint32_t movement = steps_goal - steps_position;
    if (movement > 0) {
      move_up = true;
    } else {
      move_up = false;
    }

    // Set rotation side
    PORTB ^= (-(move_up) ^ PORTB) & (1 << PIN_DIR);

    // Compute speedramp range
    uint16_t speedramps_steps = 2 * ramp_size * STEP;
    if (movement >= speedramps_steps) {
      max_speed_steps = movement - speedramps_steps;
    } else {
      max_speed_steps = 0;
    }

    // Set ISR flags
    falling = false;
    raising = true;

    i = 0;
    s = 0;

    // Initiate first interrupt
    index = 1;
    OCR1A = comparator[index];
    TCCR1 = (TCCR1 & 0xf0) | prescaler[index];
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
