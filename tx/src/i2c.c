#include <avr/io.h>
#include "i2c.h"

//------------------------------------------------------------------------------

#define TWI_TIMEOUT  60000UL // busy-wait loop limit

#define TWI_SLA_W(a)  (((a) << 1) | 0)
#define TWI_SLA_R(a)  (((a) << 1) | 1)

// TWSR status codes (prescaler = 1)
#define TWI_ST_START     0x08
#define TWI_ST_RSTART    0x10
#define TWI_ST_SLAW_ACK  0x18
#define TWI_ST_DATA_ACK  0x28
#define TWI_ST_SLAR_ACK  0x40

//------------------------------------------------------------------------------

static bool twi_wait(void) {
  uint32_t n = TWI_TIMEOUT;
  while (!(TWCR & (1 << TWINT))) {
    if (--n == 0)
      return false;
  }
  return true;
}

static bool twi_start(uint8_t sla) {
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  if (!twi_wait())
    return false;

  uint8_t st = TWSR & 0xF8;
  if (st != TWI_ST_START && st != TWI_ST_RSTART)
    return false;

  TWDR = sla;
  TWCR = (1 << TWINT) | (1 << TWEN);
  if (!twi_wait())
    return false;

  st = TWSR & 0xF8;
  return st == TWI_ST_SLAW_ACK || st == TWI_ST_SLAR_ACK;
}

static bool twi_write(uint8_t data) {
  TWDR = data;
  TWCR = (1 << TWINT) | (1 << TWEN);
  return twi_wait() && (TWSR & 0xF8) == TWI_ST_DATA_ACK;
}

static bool twi_read(uint8_t *data, bool ack) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (ack ? (1 << TWEA) : 0);
  if (!twi_wait())
    return false;

  *data = TWDR;
  return true;
}

static inline void twi_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

//------------------------------------------------------------------------------

void i2c_init(uint32_t hz) {
  TWSR = 0; // prescaler 1
  TWBR = (uint8_t)((F_CPU / hz - 16) / 2);
  TWCR = (1 << TWEN);
}

bool i2c_probe(uint8_t addr) {
  bool ret = twi_start(TWI_SLA_W(addr));
  twi_stop();
  return ret;
}

bool i2c_write(uint8_t addr, const uint8_t *hdr, uint8_t hdr_len,
               const uint8_t *data, uint16_t len) {
  bool ret = twi_start(TWI_SLA_W(addr));

  for (uint8_t i = 0; ret && i < hdr_len; i++)
    ret = twi_write(hdr[i]);

  for (uint16_t i = 0; ret && i < len; i++)
    ret = twi_write(data[i]);

  twi_stop();
  return ret;
}

bool i2c_read(uint8_t addr, const uint8_t *hdr, uint8_t hdr_len,
              uint8_t *data, uint16_t len) {
  bool ret = true;

  if (hdr_len > 0) {
    ret = twi_start(TWI_SLA_W(addr));
    for (uint8_t i = 0; ret && i < hdr_len; i++)
      ret = twi_write(hdr[i]);
  }

  if (ret)
    ret = twi_start(TWI_SLA_R(addr)); // repeated START when hdr was sent

  for (uint16_t i = 0; ret && i < len; i++)
    ret = twi_read(&data[i], i < len - 1); // NACK on the last byte

  twi_stop();
  return ret;
}
