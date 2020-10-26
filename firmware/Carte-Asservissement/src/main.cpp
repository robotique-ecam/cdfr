#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <Arduino.h>
#include <Wire.h>
#include "speedramp.hpp"

#define OBELIX
#define I2C_ADDR 0x10

#ifdef ASTERIX
#define INVERT 0
#endif

#ifdef OBELIX
#define INVERT 1
#endif

/* PORTD */
#define PIN_LEFT_STEP 5
#define PIN_LEFT_DIR 6
/* PORTB */
#define PIN_ENA 0
#define PIN_RIGHT_STEP 3
#define PIN_RIGHT_DIR 4

#define sign(N) ((N < 0) ? (1) : (0))

uint16_t left_old_steps = 0;
uint16_t right_old_steps = 0;

/* ISR variables are volatile */
volatile uint8_t left_prescaler = 0;
volatile uint8_t right_prescaler = 0;
volatile uint16_t left_steps = 0;
volatile uint16_t right_steps = 0;

ISR(TIMER0_COMPA_vect) {
  /* Disable comparator Timer0 A interrupts */
  TIMSK0 &= ~(1 << OCIE0A);
  /* Start the pulse */
  PORTD |= (1 << PIN_LEFT_STEP);
  left_steps++;
  /* Reset counter */
  TCNT1 = 0;
  /* Delay for x us by dividing clock by 8 (0.5us per count) and counting */
  TCCR0B = (TCCR0B & 0xF8) | 2;
  OCR0B = 20;
  TIMSK0 |= (1 << OCIE0B);
}

ISR(TIMER0_COMPB_vect) {
  /* Disable comparator Timer0 B interrupts */
  TIMSK0 &= ~(1 << OCIE0B);
  /* Reset counter */
  TCNT1 = 0;
  /* Restore values */
  TCCR0B = (TCCR0B & 0xF8) | left_prescaler;
  /* Stop the pulse */
  PORTD &= ~(1 << PIN_LEFT_STEP);
  /* Enable compare interrupt */
  TIMSK0 |= (1 << OCIE0A);
}

ISR(TIMER2_COMPA_vect) {
  /* Disable comparator Timer 2 A interrupts */
  TIMSK2 &= ~(1 << OCIE2A);
  /* Start the pulse */
  PORTB |= (1 << PIN_RIGHT_STEP);
  right_steps++;
  /* Reset counter */
  TCNT1 = 0;
  /* Delay for x us by dividing clock by 8 (0.5us per count) and counting */
  TCCR2B = (TCCR2B & 0xF8) | 2;
  OCR2B = 20;
  TIMSK2 |= (1 << OCIE2B);
}

ISR(TIMER2_COMPB_vect) {
  /* Disable comparator Timer 2 B interrupts */
  TIMSK2 &= ~(1 << OCIE2B);
  /* Reset counter */
  TCNT1 = 0;
  /* Restore values */
  TCCR2B = (TCCR2B & 0xF8) | right_prescaler;
  /* Stop the pulse */
  PORTB &= ~(1 << PIN_RIGHT_STEP);
  /* Enable compare interrupt */
  TIMSK2 |= (1 << OCIE2A);
}

void onReceive(int n) {
  bool enabled;
  bool left_dir;
  bool right_dir;
  uint8_t buffer[3];
  uint16_t left_index;
  uint16_t right_index;

  /* Disable interrupts */
  cli();

  /* Fetch packet from buffer */
  if (Wire.available() >= 4) {
    uint8_t cmd = Wire.read();
    for (int i = 0; i < 3; i++) {
      buffer[i] = Wire.read();
    }
    /* Unpack data */
    enabled = 0x80 & buffer[0];
    left_dir = buffer[0] & 0x20;
    right_dir = buffer[0] & 0x10;
    left_index = (0x0F & buffer[0] << 8) | (buffer[1] & 0xFC);
    right_index = (0x03 & buffer[1] << 8) | buffer[2];
    left_prescaler = prescaler[left_index];
    right_prescaler = prescaler[right_index];

    TCNT0 = TCNT2 = 0;
    left_old_steps = left_steps;
    left_old_steps = left_steps;
    left_steps = right_steps = 0;

    PORTB ^= (-(enabled) ^ PORTB) & (1 << PIN_ENA);
    PORTB ^= (-(right_dir ^ INVERT) ^ PORTB) & (1 << PIN_RIGHT_DIR);
    PORTD ^= (-(left_dir ^ INVERT) ^ PORTD) & (1 << PIN_LEFT_DIR);

    TCCR0B = (TCCR0B & 0xF8) | left_prescaler;
    OCR0A = comparator[left_index];
    TCCR2B = (TCCR0B & 0xF8) | right_prescaler;
    OCR2A = comparator[right_index];
  }

  /* Enable interrupts */
  sei();
}

void onRequest() {
  uint8_t buffer[4];

  if (Wire.available()) {
    Wire.read();
  }

  buffer[0] = left_old_steps >> 8;
  buffer[1] = left_old_steps & 0xFF;
  buffer[2] = right_old_steps >> 8;
  buffer[3] = right_old_steps & 0xFF;
  Wire.write(buffer, 4);
}

void setup() {
  /* Setup IO */
  PORTB = PORTD = 0;
  DDRB |= (1 << PIN_RIGHT_DIR) | (1 << PIN_RIGHT_STEP) | (1 << PIN_ENA);
  DDRD |= (1 << PIN_LEFT_DIR) | (1 << PIN_LEFT_STEP);
  PORTB |= (1 << PIN_ENA);

  Wire.begin(I2C_ADDR);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
}

void loop() {}
