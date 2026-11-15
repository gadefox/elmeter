#ifndef I2C_H
#define I2C_H

#include <stdbool.h>
#include <stdint.h>

//------------------------------------------------------------------------------

// Configure TWI master at the given SCL frequency (e.g. 100000).
void i2c_init(uint32_t hz);

// True if a device ACKs its 7-bit address.
bool i2c_probe(uint8_t addr);

// Write: START, SLA+W, hdr[hdr_len], data[len], STOP.
// hdr is typically a register / memory address, hdr_len may be 0.
bool i2c_write(uint8_t addr, const uint8_t *hdr, uint8_t hdr_len,
               const uint8_t *data, uint16_t len);

// Read: [START, SLA+W, hdr[hdr_len], repeated START,] SLA+R, data[len], STOP.
// With hdr_len == 0 it is a plain read from the current address pointer.
bool i2c_read(uint8_t addr, const uint8_t *hdr, uint8_t hdr_len,
              uint8_t *data, uint16_t len);

#endif // I2C_H
