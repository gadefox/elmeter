#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include "manchtx.h"
#include "util.h"

//------------------------------------------------------------------------------

#define TX_PIN   0
#define TX_PORT  PORTB
#define TX_DDR   DDRB

#define MANCH_PREAMBLE_BYTES  3       // 0xAA each
#define MANCH_SYNC            0x2DD4  // sync word after preamble

#define MANCH_TX_BITRATE  2000UL
#define MANCH_TX_REPEAT   3
#define MANCH_TX_GAP_MS   20

#define MANCH_MAX_FRAME  (MANCH_PREAMBLE_BYTES + MANCH_MAX_PAYLOAD + 4)

//------------------------------------------------------------------------------

static uint8_t s_frame[MANCH_MAX_FRAME];
static bool s_half;
static uint8_t s_nbytes;
static uint8_t s_bytepos;
static uint8_t s_bitmask;

//------------------------------------------------------------------------------

static uint16_t manch_tx_build(const uint8_t *data, uint8_t len) {
  if (len == 0 || len > MANCH_MAX_PAYLOAD)
    return 0;

  uint8_t n = 0;

  for (uint8_t i = 0; i < MANCH_PREAMBLE_BYTES; i++)
    s_frame[n++] = 0xAA;

  s_frame[n++] = MANCH_SYNC >> 8;
  s_frame[n++] = MANCH_SYNC & 0xFF;

  uint8_t crc = crc8(0, len);
  s_frame[n++] = len;

  for (uint8_t i = 0; i < len; i++) {
    s_frame[n++] = data[i];
    crc = crc8(crc, data[i]);
  }

  s_frame[n++] = crc;
  return n;
}

static void manch_tx_start(uint16_t len) {
  s_bytepos = 0;
  s_bitmask = 0x80;
  s_half = false;
  s_nbytes = len;
}

//------------------------------------------------------------------------------

static void manch_timer_on(void) {
  TCCR1A = 0;
  TCNT1  = 0;
  OCR1A  = (F_CPU / 8 / (2 * MANCH_TX_BITRATE)) - 1;
  TIFR1  = (1 << OCF1A);
  TIMSK1 = (1 << OCIE1A);
  TCCR1B = (1 << WGM12) | (1 << CS11);
}

static inline void manch_timer_off(void) {
  TCCR1B = 0;                            // stop
  TIMSK1 &= ~(1 << OCIE1A);
  TX_PORT &= ~(1 << TX_PIN);                 // idle = low
}

ISR(TIMER1_COMPA_vect) {
  if (s_bytepos >= s_nbytes) {
    manch_timer_off();
    return;
  }

  bool bit = s_frame[s_bytepos] & s_bitmask;
  bool out;

  if (s_half) {
    out = bit;
    s_half = false;

    s_bitmask >>= 1;
    if (!s_bitmask) {
      s_bitmask = 0x80;
      s_bytepos++;
    }
  } else {
    out = !bit;
    s_half = true;
  }

  if (out)
    TX_PORT |= (1 << TX_PIN);
  else
    TX_PORT &= ~(1 << TX_PIN);
}

//------------------------------------------------------------------------------

void manch_tx_init(void) {
  TX_DDR |= (1 << TX_PIN);
  TX_PORT &= ~(1 << TX_PIN);
}

bool manch_tx_send(const uint8_t *data, uint8_t len) {
  uint16_t frame_bytes = manch_tx_build(data, len);
  if (!frame_bytes)
    return false;

  for (uint8_t i = 0; i < MANCH_TX_REPEAT; i++) {
    manch_tx_start(frame_bytes);
    __asm__ volatile("" ::: "memory");

    manch_timer_on();
    while (TCCR1B)
      /* nop */ ;

    _delay_ms(MANCH_TX_GAP_MS);
  }

  return true;
}
