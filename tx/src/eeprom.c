#include <util/delay.h>
#include "i2c.h"
#include "eeprom.h"

//------------------------------------------------------------------------------

// Chip configuration (see datasheet):
#define EEPROM_I2C_ADDR    0x50 // 0x50 | A2 A1 A0
#define EEPROM_ADDR_BYTES  2

// Internal write cycle is max 5 ms (typ.), poll for ACK up to 10 ms.
#define EEPROM_POLL_STEP_US  100
#define EEPROM_POLL_COUNT    100

//------------------------------------------------------------------------------

static inline uint8_t eeprom_make_hdr(uint16_t addr, uint8_t *hdr) {
  hdr[0] = addr >> 8;
  hdr[1] = addr & 0xFF;
  return EEPROM_ADDR_BYTES;
}

// ACK polling: the chip does not ACK its address while writing internally.
static bool eeprom_wait_ready(void) {
  for (uint8_t i = 0; i < EEPROM_POLL_COUNT; i++) {
    if (i2c_probe(EEPROM_I2C_ADDR))
      return true;
    _delay_us(EEPROM_POLL_STEP_US);
  }
  return false;
}

static bool eeprom_write_page(uint16_t addr, const uint8_t *data,
                              uint8_t length) {
  uint8_t hdr[EEPROM_ADDR_BYTES];
  uint8_t hdr_len = eeprom_make_hdr(addr, hdr);

  if (!i2c_write(EEPROM_I2C_ADDR, hdr, hdr_len, data, length))
    return false;

  return eeprom_wait_ready();
}

//------------------------------------------------------------------------------

bool eeprom_init(void) {
  return i2c_probe(EEPROM_I2C_ADDR);
}

bool eeprom_read(uint16_t addr, uint8_t *data, uint16_t length) {
  if (length == 0 || (uint32_t)addr + length > EEPROM_SIZE)
    return false;

  uint8_t hdr[EEPROM_ADDR_BYTES];
  uint8_t hdr_len = eeprom_make_hdr(addr, hdr);
  return i2c_read(EEPROM_I2C_ADDR, hdr, hdr_len, data, length);
}

bool eeprom_write(uint16_t addr, const uint8_t *data, uint16_t length) {
  if ((uint32_t)addr + length > EEPROM_SIZE)
    return false;

  // A write must not cross a page boundary (the pointer would wrap inside
  // the page), so split the data at page edges.
  while (length > 0) {
    uint16_t room = EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE);
    uint8_t n = length < room ? length : room;

    if (!eeprom_write_page(addr, data, n))
      return false;

    addr += n;
    data += n;
    length -= n;
  }
  return true;
}
