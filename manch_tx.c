#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include "manch_tx.h"

//------------------------------------------------------------------------------

#define MANCH_TX_HALF  (1 << 0)  // 0: first half of the bit, 1: second half
#define MANCH_TX_BUSY  (1 << 1)

#define MANCH_TX_PREAMBLE_BYTES 3  // 0xAA each
#define MANCH_TX_MAX_FRAME      (MANCH_TX_PREAMBLE_BYTES + 2 + 1 + \
                                 MANCH_MAX_PAYLOAD + 1)

#define MANCH_TX_BITRATE  2000UL
#define MANCH_TX_REPEAT   3
#define MANCH_TX_GAP_MS   20

//------------------------------------------------------------------------------

static uint8_t s_pinmask;
static volatile uint8_t s_flags;

static uint8_t s_frame[MANCH_TX_MAX_FRAME];
static uint16_t s_nbits;
static uint16_t s_bitpos;

//------------------------------------------------------------------------------

static uint8_t manch_crc8(uint8_t crc, uint8_t init) {
  crc ^= init;
  for (uint8_t i = 0; i < 8; i++)
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);

  return crc;
}

static uint16_t manch_tx_build(const uint8_t *data, uint8_t len) {
  if (s_flags & MANCH_TX_BUSY)
    return -1;
  if (len == 0 || len > MANCH_MAX_PAYLOAD)
    return -1;

  uint8_t n = 0;

  for (uint8_t i = 0; i < MANCH_TX_PREAMBLE_BYTES; i++)
    s_frame[n++] = 0xAA;

  s_frame[n++] = MANCH_SYNC >> 8;
  s_frame[n++] = MANCH_SYNC & 0xFF;

  uint8_t crc = manch_crc8(0, len);
  s_frame[n++] = len;

  for (uint8_t i = 0; i < len; i++) {
    s_frame[n++] = data[i];
    crc = manch_crc8(crc, data[i]);
  }

  s_frame[n++] = crc;
  return (uint16_t)n * 8;  // bits
}

static void manch_tx_start(uint16_t size_bits) {
  s_nbits = size_bits;
  s_bitpos = 0;

  __asm__ volatile("" ::: "memory");  // frame must be complete before BUSY
  s_flags = MANCH_TX_BUSY;
}

static inline bool manch_tx_is_busy(void) {
  return (s_flags & MANCH_TX_BUSY);
}

static bool manch_tx_tick(manch_tx_t *t) {
  if (!manch_tx_is_busy())
    return false;

  if (s_bitpos >= s_nbits) {
    s_flags = 0;
    return false;
  }

  bool bit = s_frame[s_bitpos >> 3] & (0x80 >> (s_bitpos & 7));

  if (s_flags & MANCH_TX_HALF) {
    s_bitpos++;
    s_flags &= ~MANCH_TX_HALF;
    return bit;
  }

  s_flags |= MANCH_TX_HALF;
  return !bit;
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

static void manch_timer_off(void) {
  TCCR1B = 0;                            // stop
  TIMSK1 &= ~(1 << OCIE1A);
  TX_PORT &= ~s_pinmask;                 // idle = low
}

ISR(TIMER1_COMPA_vect) {
  if (manch_tx_tick(&tx))
    TX_PORT |= s_pinmask;
  else
    TX_PORT &= ~s_pinmask;
}

//------------------------------------------------------------------------------

void manch_tx_init(uint8_t pindata) {
  s_nbits = 0;
  s_bitpos = 0;
  s_flags = 0;

  s_pinmask = (uint8_t)(1 << pindata);
  TX_DDR |= s_pinmask;
  TX_PORT &= ~s_pinmask;
}

bool manch_tx_send(const uint8_t *data, uint8_t len) {
  uint16_t size_bits = manch_tx_build(data, len);
  if (!size_bits)
    return false;

  for (uint8_t i = 0; i < MANCH_TX_REPEAT; i++) {
    manch_tx_start(size_bits);

    manch_timer_on();
    while (manch_tx_is_busy(&tx))
      /* nop */ ;
    manch_timer_off();

    _delay_ms(MANCH_TX_GAP_MS);
  }

  return true;
}
