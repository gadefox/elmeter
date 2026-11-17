#ifndef METER_H
#define METER_H

#include <stdbool.h>
#include <stdint.h>
#include "ds1307.h"

//------------------------------------------------------------------------------

typedef enum {
  METER_NO_TIME,   // T0 / clock invalid, waiting for meter_set_time()
  METER_RUNNING
} meter_state_t;

typedef struct {
  uint32_t start;                      // bucket of cnt[0]
  uint8_t  num;                        // valid windows, 0..METER_PAGE_WINDOWS-1
  uint16_t cnt[METER_PAGE_WINDOWS];    // pulses per window, UNKNOWN past n
} meter_page_t;

//------------------------------------------------------------------------------

// Init I2C, DS1307, EEPROM, INT0 (PD2, falling edge) and Timer0, enables
// interrupts. False only when a chip does not respond; with invalid time it
// returns true and meter_state() stays METER_NO_TIME.
bool meter_init(void);

// Call from the main loop as often as possible (> 4x per second).
void meter_poll(void);

meter_state_t meter_get_state(void);

// Set RTC time. The very first call (no valid T0) behaves as meter_reset().
// Later calls keep T0; time before T0 is rejected.
bool meter_set_time(const ds1307_time_t *tm);

// Erase the EEPROM history, set RTC time and a new T0.
bool meter_reset(const ds1307_time_t *tm);

// Write the closed windows from the DS1307 RAM buffer to the EEPROM now.
bool meter_flush(void);

//------------------------------------------------------------------------------

// Pulses in the currently open window.
uint16_t meter_current(void);

uint32_t meter_t0(void);

// Page index the next flush writes to; newest data is at (next - 1) % PAGES.
uint8_t meter_next_page(void);

// Read and decode one EEPROM page. False when empty or CRC mismatch.
bool meter_read_page(uint8_t idx, meter_page_t *page);

// Decode closed windows that are still only in the DS1307 RAM buffer.
void meter_read_buffer(meter_page_t *page);

// Time helpers (RTC time is treated as UTC).
uint32_t meter_time_to_epoch(const ds1307_time_t *t);
void meter_epoch_to_time(uint32_t epoch, ds1307_time_t *t);
uint32_t meter_bucket_to_epoch(uint32_t bucket);

#endif // METER_H
