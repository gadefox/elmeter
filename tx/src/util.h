#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>

extern const uint8_t crc8_table[256] PROGMEM;

//------------------------------------------------------------------------------

static inline uint8_t crc8(uint8_t crc, uint8_t data) {
  return pgm_read_byte(&crc8_table[crc ^ data]);
}

uint8_t crc8_calc(uint8_t seed, const uint8_t *data, uint8_t len);

#endif // UTIL_H
