#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <TinyWireS.h>

#define TRIG_PIN 4
#define ECHO_PIN 3
#define DEVICE_ADDR 0x20
#define LED_PIN 0

#define EEPROM_I2C_ADDR 0xad
#define I2C_ADDR_DEFAULT 0x20

////////////////// FLAGS ////////////////////////////

// FLAGS are volatile to be within ISR scope

#define SFRCV 0  // Value has been computed sucessfuly
#define SFWAIT 1 // Timer has stated, waiting for low state
#define SFWOVF 2 // Missed low state, must retry
#define SFTRIG 3 // HCSR0x has been triggered

#define FILTER_WINDOW 3
#define swap(a, b)                                                                                                                                                                 \
  a ^= b;                                                                                                                                                                          \
  b ^= a;                                                                                                                                                                          \
  a ^= b;
#define sort(a, b)                                                                                                                                                                 \
  if (a > b) {                                                                                                                                                                     \
    swap(a, b);                                                                                                                                                                    \
  }

volatile uint8_t flags;

uint8_t distance = 0;
uint8_t i2c_addr = I2C_ADDR_DEFAULT;
uint8_t dutyCycle = 0;

/* Median filter */
uint8_t readings[FILTER_WINDOW];
uint8_t index = 0;

////////////////// ROUTINES //////////////////////////

void setup() {
  /* Set Inputs/Outputs */
  PORTB = 0;
  DDRB |= (1 << TRIG_PIN) | (0 << ECHO_PIN) | (1 << DDB1);

  /* Setup PCINT3 REGISTERS AND ISR */
  GIMSK |= (1 << PCIE);
  PCMSK |= (1 << ECHO_PIN);

  /* Enable timer1 overflow interrupts */
  TIMSK |= (1 << TOIE1);

  /* set timer0 A and B on PWM output and non-inverser */
  TCCR0A |= (1 << WGM01) | (1 << WGM00) | (1 << COM0A1) | (1 << COM0B1);

/* setting for a 488Hz PWM frequency
f_pwm = f_ATtiny / (256 * prescaler) */
#if F_CPU == 1000000UL
  // prescaler to 8
  TCCR0B |= (1 << CS01);
#elif F_CPU == 8000000UL
  // prescaler to 64
  TCCR0B |= (1 << CS01) | (1 << CS00);
#elif F_CPU == 16000000UL
  // prescaler to 256, here no prescaler so f_pwm to 244Hz
  TCCR0B |= (1 << CS02);
#else
#error("Check datasheet for TCCR0B register content")
#endif
}

void generate_trig_pulse() {
  /* Generate a 10us pulse on TRIG PIN */
  PORTB |= (1 << TRIG_PIN);
  _delay_us(10);
  PORTB &= ~(1 << TRIG_PIN);
  flags = (1 << SFTRIG);
}

void start_counter() {
  // reset count register
  TCNT1 = 0;
// Prescaler to match 64us which is closest value to 58 us (1cm)
#if F_CPU == 1000000UL
  // prescaler to 1*64
  TCCR1 |= (1 << CS12) | (1 << CS11) | (1 << CS10);
#elif F_CPU == 8000000UL
  // prescaler to 8*64 = 512
  TCCR1 |= (1 << CS13) | (1 << CS11);
#elif F_CPU == 16000000UL
  // prescaler to 16*64 = 1024
  TCCR1 |= (1 << CS13) | (1 << CS11) | (1 << CS10);
#else
#error("Check datasheet for TCCR1 register content")
#endif
}

void stop_counter() {
  /* Stop counter by disabling timer 1 prescaler CS */
  TCCR1 &= ~(0x0f);
}

void transmit(uint8_t n) {
  /* Callback for I2C byte request */
  TinyWireS.send(0xDE);
}

void pwm_PB0(uint8_t dutyCycle) {
  /* Set duty cycle of the pwm on the timer0B for the PB0 output
     50cm ~ 0% and 5cm ~ 8% */
  if (dutyCycle < 0.5 || dutyCycle > 20) {
    dutyCycle = 0;
  }
  OCR0B = dutyCycle;
}

uint8_t median(uint8_t *mesurements) {
  sort(mesurements[0], mesurements[2]);
  sort(mesurements[0], mesurements[1]);
  return mesurements[1];
}

////////////////// INTERUPTS HANDLING ///////////////

ISR(PCINT0_vect) {
  /* if PIN3 in HIGH state */
  if (PINB & _BV(3)) {
    start_counter();
    flags = (1 << SFWAIT);
  } else {
    stop_counter();
    flags = (1 << SFRCV);
  }
}

ISR(TIMER1_OVF_vect) {
  stop_counter();
  flags |= (1 << SFWOVF) | (1 << SFRCV);
}

////////////////// MAIN  ///////////////////////////

int main(int argc, char const *argv[]) {
  // TODO : For future, read addr in EEPROM
  // uint8_t device_addr = eeprom_read_byte(EEPROM_I2C_ADDR);

  setup();
  // setup_from_eeprom();

  // Init I2C
  TinyWireS.begin(i2c_addr);
  TinyWireS.onReceive(transmit);

  // Enable interupts
  sei();

  while (1) {

    TinyWireS_stop_check();
  }

  return 0;
}
