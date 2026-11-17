#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include "led.h"

//------------------------------------------------------------------------------

// Hardcoded pins, active high. Adjust to your board.
#define LED1_PORT  PORTB
#define LED1_DDR   DDRB
#define LED1_BIT   (1 << PB0)

#define LED2_PORT  PORTB
#define LED2_DDR   DDRB
#define LED2_BIT   (1 << PB1)

#define LED_HB     0xFF         // heartbeat pseudo-code
#define LED_PAUSE  10           // ticks of pause after the blink sequence

//------------------------------------------------------------------------------

typedef struct {
  volatile uint8_t code;        // 0 = off, 1..N = N blinks, LED_HB = heartbeat
  uint8_t          pos;
} led_t;

static led_t led1, led2;

//------------------------------------------------------------------------------

static inline void led_set(led_t *l, uint8_t code) {
  if (l->code != code) {
    l->pos = 0;
    l->code = code;
  }
}

static bool led_step(led_t *l) {
  uint8_t code = l->code;
  bool on;
  uint8_t len;

  if (code == 0)
    return false;

  if (code == LED_HB) {
    on = l->pos < 2;            // 200 ms flash, 1 Hz
    len = 10;
  } else {
    on = l->pos < 2 * code && !(l->pos & 1);
    len = 2 * code + LED_PAUSE;
  }

  if (++l->pos >= len)
    l->pos = 0;
  return on;
}

//------------------------------------------------------------------------------

void led_init(void) {
  LED1_DDR |= LED1_BIT;
  LED2_DDR |= LED2_BIT;
  LED1_PORT &= ~LED1_BIT;
  LED2_PORT &= ~LED2_BIT;
}

void led_tick(void) {
  if (led_step(&led1))
    LED1_PORT |= LED1_BIT;
  else
    LED1_PORT &= ~LED1_BIT;

  if (led_step(&led2))
    LED2_PORT |= LED2_BIT;
  else
    LED2_PORT &= ~LED2_BIT;
}

void led_boot(uint8_t phase) {
  led_set(&led1, phase);
  led_set(&led2, 0);
}

void led_run(void) {
  led_set(&led1, LED_HB);
  led_set(&led2, 0);
}

void led_error(uint8_t code) {
  led_set(&led1, LED_HB);
  led_set(&led2, code);
}

void led_fatal(uint8_t code) {
  cli();

  for (;;) {
    for (uint8_t i = 0; i < code; i++) {
      LED1_PORT |= LED1_BIT;
      LED2_PORT |= LED2_BIT;
      _delay_ms(150);

      LED1_PORT &= ~LED1_BIT;
      LED2_PORT &= ~LED2_BIT;
      _delay_ms(150);
    }
    _delay_ms(1200);
  }
}
