#ifndef LED_H
#define LED_H

#include <stdint.h>

//------------------------------------------------------------------------------

#define LED_TICK_MS  100        // period of led_tick()

void led_init(void);
void led_tick(void);            // call every LED_TICK_MS (ISR-safe)

void led_boot(uint8_t phase);   // init phase 1..N, LED1 blinks N times
void led_run(void);             // all OK, LED1 heartbeat
void led_error(uint8_t code);   // recoverable error 1..N, LED2 blinks N times
void led_fatal(uint8_t code) __attribute__((noreturn));

#endif // LED_H
