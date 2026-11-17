#ifndef DS1307_H
#define DS1307_H

#include <stdbool.h>
#include <stdint.h>

//------------------------------------------------------------------------------

#define DS1307_RAM_SIZE  56     // user RAM, registers 0x08..0x3F

typedef struct {
  uint8_t  sec;    // 0..59
  uint8_t  min;    // 0..59
  uint8_t  hour;   // 0..23 (24h mode)
  uint8_t  wday;   // 1..7
  uint8_t  day;    // 1..31
  uint8_t  month;  // 1..12
  uint16_t year;   // 2000..2099
} ds1307_time_t;

//------------------------------------------------------------------------------

// Configure TWI (100 kHz). Returns false if the chip does not respond.
bool ds1307_init(void);

// False when the oscillator is halted (CH bit), i.e. time is not valid,
// e.g. after battery loss. ds1307_set_time() starts the oscillator.
bool ds1307_is_running(void);

bool ds1307_set_time(const ds1307_time_t *tm);
bool ds1307_get_time(ds1307_time_t *tm);

// Battery-backed user RAM (56 bytes), offset 0..55.
bool ds1307_ram_write(uint8_t offset, const uint8_t *data, uint8_t length);
bool ds1307_ram_read(uint8_t offset, uint8_t *data, uint8_t length);

#endif // DS1307_H
