#include "i2c.h"
#include "ds1307.h"

//------------------------------------------------------------------------------

#define DS1307_ADDR  0x68         // fixed 7-bit I2C address

#define DS1307_REG_SEC  0x00
#define DS1307_REG_RAM  0x08

#define DS1307_BIT_CH   (1 << 7)  // clock halt, bit 7 of seconds register
#define DS1307_BIT_12H  (1 << 6)  // 12h mode flag in hours register

//------------------------------------------------------------------------------

static inline uint8_t ds1307_bcd2bin(uint8_t val) {
  return (val >> 4) * 10 + (val & 0x0F);
}

static inline uint8_t ds1307_bin2bcd(uint8_t val) {
  return ((val / 10) << 4) | (val % 10);
}

static inline bool ds1307_read_regs(uint8_t reg, uint8_t *data, uint8_t length) {
  return i2c_read(DS1307_ADDR, &reg, 1, data, length);
}

static inline bool ds1307_write_regs(uint8_t reg, const uint8_t *data, uint8_t length) {
  return i2c_write(DS1307_ADDR, &reg, 1, data, length);
}

//------------------------------------------------------------------------------

bool ds1307_init(void) {
  return i2c_probe(DS1307_ADDR);
}

bool ds1307_is_running(void) {
  uint8_t sec;
  if (!ds1307_read_regs(DS1307_REG_SEC, &sec, 1))
    return false;

  return !(sec & DS1307_BIT_CH);
}

bool ds1307_set_time(const ds1307_time_t *tm) {
  uint8_t buf[7];
  buf[0] = ds1307_bin2bcd(tm->sec) & ~DS1307_BIT_CH; // CH = 0 -> oscillator on
  buf[1] = ds1307_bin2bcd(tm->min);
  buf[2] = ds1307_bin2bcd(tm->hour) & ~DS1307_BIT_12H; // 24h mode
  buf[3] = ds1307_bin2bcd(tm->wday);
  buf[4] = ds1307_bin2bcd(tm->day);
  buf[5] = ds1307_bin2bcd(tm->month);
  buf[6] = ds1307_bin2bcd(tm->year - 2000);

  return ds1307_write_regs(DS1307_REG_SEC, buf, sizeof(buf));
}

bool ds1307_get_time(ds1307_time_t *tm) {
  uint8_t buf[7];
  if (!ds1307_read_regs(DS1307_REG_SEC, buf, sizeof(buf)))
    return false;

  tm->sec   = ds1307_bcd2bin(buf[0] & ~DS1307_BIT_CH);
  tm->min   = ds1307_bcd2bin(buf[1]);
  tm->hour  = ds1307_bcd2bin(buf[2] & 0x3F);
  tm->wday  = ds1307_bcd2bin(buf[3]);
  tm->day   = ds1307_bcd2bin(buf[4]);
  tm->month = ds1307_bcd2bin(buf[5]);
  tm->year  = ds1307_bcd2bin(buf[6]) + 2000;
  return true;
}

bool ds1307_ram_write(uint8_t offset, const uint8_t *data, uint8_t length) {
  // The register pointer wraps 0x3F -> 0x00, so reject overruns.
  if ((uint16_t)offset + length > DS1307_RAM_SIZE)
    return false;

  return ds1307_write_regs(DS1307_REG_RAM + offset, data, length);
}

bool ds1307_ram_read(uint8_t offset, uint8_t *data, uint8_t length) {
  if ((uint16_t)offset + length > DS1307_RAM_SIZE || length == 0)
    return false;

  return ds1307_read_regs(DS1307_REG_RAM + offset, data, length);
}
