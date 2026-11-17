#ifndef EEPROM_H
#define EEPROM_H

#include <stdbool.h>
#include <stdint.h>

//------------------------------------------------------------------------------

// Chip configuration (see datasheet):
#define EEPROM_SIZE       4096
#define EEPROM_PAGE_SIZE  32
#define EEPROM_PAGE_MAX   (EEPROM_SIZE / EEPROM_PAGE_SIZE)

//------------------------------------------------------------------------------

bool eeprom_init(void);
bool eeprom_read(uint16_t addr, uint8_t *data, uint16_t length);
bool eeprom_write(uint16_t addr, const uint8_t *data, uint16_t length);

#endif // EEPROM_H
