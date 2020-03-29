#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define PIN 1

void setup() {
  /* Setup IO */
  PORTB = 0;
  DDRB |= (1 << PIN);
}

void delay() {
  asm("nop\n"
      "nop\n"
      "nop\n"
      "nop\n"
      "nop\n"
      "nop\n"
      "nop\n"
      "nop\n"
      "nop\n"

      "nop\n"
      "nop\n"
      "nop\n"
      "nop\n"
      "nop\n"
      "nop\n"
      "nop\n"
      "nop\n"
      "nop\n");
}

int main(int argc, char const *argv[]) {

  setup();

  while (1) {
    PORTB |= (1 << PIN);
    delay();
    asm("nop\n"
        "nop\n");
    PORTB &= ~(1 << PIN);
    delay();
  }
  return 0;
}
