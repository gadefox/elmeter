#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>
#include <string.h>
#include <util/atomic.h>
#include "i2c.h"
#include "eeprom.h"
#include "meter.h"
#include "util.h"

//------------------------------------------------------------------------------

#define METER_DEBOUNCE_MS  30UL
#define METER_POLL_MS      250UL

// Timer0 overflow tick, prescaler 1024 (16384 us at 16 MHz).
#define METER_TICK_US         (262144000UL / (F_CPU / 1000UL))
#define METER_DEBOUNCE_TICKS  ((uint8_t)((METER_DEBOUNCE_MS * 1000UL + METER_TICK_US - 1) / METER_TICK_US + 1))
#define METER_POLL_TICKS      ((uint8_t)(METER_POLL_MS * 1000UL / METER_TICK_US))

static volatile uint8_t s_tick;
static volatile uint16_t s_cnt;
static uint8_t s_last_pulse;
static uint8_t s_last_poll;

//------------------------------------------------------------------------------

// 1 pulse = 1 Wh, window = 5 min. Window number ("bucket") counts windows
// since T0, T0 is stored in the DS1307 RAM and is aligned to METER_WINDOW_SEC.
#define METER_WINDOW_SEC     300u    // 5 minutes
#define METER_COUNT_MAX      0xFFE
#define METER_COUNT_UNKNOWN  0xFFF

#define METER_PAGES         EEPROM_PAGE_MAX
#define METER_PAGE_WINDOWS  18u     // 18 x 12 bit = 27 B in one 32 B EEPROM page
#define METER_DATA_BYTES    (METER_PAGE_WINDOWS * 3 / 2)

//------------------------------------------------------------------------------

#define METER_CRC_SEED  0xFF

// One block = one EEPROM page = image of the closed windows in DS1307 RAM.
typedef struct {
  uint8_t start[3];                 // bucket of the first window (LE)
  uint8_t num;                      // number of closed windows
  uint8_t data[METER_DATA_BYTES];   // 12 bit values, unused bytes 0xFF
  uint8_t crc;                      // CRC-8 over everything above
} __attribute__((packed)) meter_blk_t;

_Static_assert(sizeof(meter_blk_t) == EEPROM_PAGE_SIZE, "blk != EEPROM page");

// Layout of the DS1307 RAM (offset 0..39).
typedef struct {
  uint32_t    t0;                   // epoch of window 0, set via UART
  uint8_t     t0_crc;
  uint16_t    cur;                  // open window, last saved value
  uint8_t     cur_chk;              // current check
  meter_blk_t blk;
} __attribute__((packed)) meter_ram_t;

// NOTE: RAM layout offsets. If meter_ram_t changes (members added, removed
// or reordered), these macros must be updated.
#define METER_RAM_CUR_OFFSET   offsetof(meter_ram_t, cur)
#define METER_RAM_CUR_SIZE     (offsetof(meter_ram_t, blk) - METER_RAM_CUR_OFFSET)
#define METER_RAM_HEADER_SIZE  METER_RAM_CUR_OFFSET

_Static_assert(sizeof(meter_ram_t) <= DS1307_RAM_SIZE, "RAM layout too big");

static meter_ram_t s_ram;
static uint8_t s_next_page;

//------------------------------------------------------------------------------

#define METER_FLAG_T0_VALID  (1 << 0)
#define METER_FLAG_RUNNING   (1 << 1)
#define METER_FLAG_DIRTY     (1 << 2)

static volatile uint8_t s_flags;

//------------------------------------------------------------------------------

ISR(TIMER0_OVF_vect) {
  s_tick++;
}

ISR(INT0_vect) {
  uint8_t now = s_tick;
  if ((uint8_t)(now - s_last_pulse) < METER_DEBOUNCE_TICKS)
    return;

  s_last_pulse = now;
  s_cnt++;
  s_flags |= METER_FLAG_DIRTY;
}

static inline uint16_t meter_swap_cnt(uint16_t cnt) {
  uint16_t ret;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    ret = s_cnt;
    s_cnt = cnt;
    s_flags &= ~METER_FLAG_DIRTY;
  }
  return ret;
}

//------------------------------------------------------------------------------

static inline void meter_set_flag(uint8_t mask) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    s_flags |= mask;
  }
}

//------------------------------------------------------------------------------

static inline uint8_t meter_flag_is_dirty(void) {
  return (s_flags & METER_FLAG_DIRTY);
}

static inline void meter_set_flag_dirty(void) {
  meter_set_flag(METER_FLAG_DIRTY);
}

//------------------------------------------------------------------------------

static inline uint8_t meter_flag_is_t0_valid(void) {
  return (s_flags & METER_FLAG_T0_VALID);
}

//------------------------------------------------------------------------------

static inline uint8_t meter_flag_is_running(void) {
  return (s_flags & METER_FLAG_RUNNING);
}

static inline void meter_set_flag_running(void) {
  meter_set_flag(METER_FLAG_RUNNING);
}

//------------------------------------------------------------------------------

static inline uint8_t meter_blk_calc_crc(const meter_blk_t *blk) {
  return crc8_calc(METER_CRC_SEED, (const uint8_t *)blk, sizeof(meter_blk_t) - 1);
}

static bool meter_blk_is_valid(const meter_blk_t *blk) {
  if (blk->num > METER_PAGE_WINDOWS)
    return false;

  uint8_t crc = meter_blk_calc_crc(blk);
  return (crc == blk->crc);
}

//------------------------------------------------------------------------------

static inline uint32_t meter_blk_get_start(const meter_blk_t *blk) {
  return (uint32_t)blk->start[0] |
        ((uint32_t)blk->start[1] << 8) |
        ((uint32_t)blk->start[2] << 16);
}

static inline void meter_blk_set_start(meter_blk_t *blk, uint32_t start) {
  blk->start[0] = start & 0xFF;
  blk->start[1] = (start >> 8) & 0xFF;
  blk->start[2] = (start >> 16) & 0xFF;
}

//------------------------------------------------------------------------------
//  ptr[0]        ptr[1]        ptr[2]
// AAAAAAAA      AAAABBBB      BBBBBBBB
// A[11:4]    A[3:0] B[11:8]    B[7:0]

static inline void meter_set_cnt(uint8_t *data, uint8_t idx, uint16_t cnt) {
  uint8_t *ptr = data + (idx >> 1) * 3;

  if (idx & 1) {
    ptr[1] = (ptr[1] & 0xF0) | ((cnt >> 8) & 0x0F);
    ptr[2] = cnt & 0xFF;
  } else {
    ptr[0] = (cnt >> 4) & 0xFF;
    ptr[1] = (ptr[1] & 0x0F) | ((cnt << 4) & 0xF0);
  }
}

static inline uint16_t meter_get_cnt(const uint8_t *data, uint8_t idx) {
  const uint8_t *ptr = data + (idx >> 1) * 3;

  if (idx & 1)
    return ((uint16_t)(ptr[1] & 0x0F) << 8) | ptr[2];

  return ((uint16_t)ptr[0] << 4) | (ptr[1] >> 4);
}

//------------------------------------------------------------------------------

static inline uint8_t meter_calc_cur_chk(uint16_t cur) {
  return (cur & 0xFF) ^ (cur >> 8) ^ 0x5A;  // seed=0x5A
}

static inline void meter_set_cur(uint16_t cur) {
  s_ram.cur = cur;
  s_ram.cur_chk = meter_calc_cur_chk(cur);
}

//------------------------------------------------------------------------------

uint32_t meter_time_to_epoch(const ds1307_time_t *tm) {
  // days from civil (H. Hinnant), years 2000..2099
  uint32_t y = tm->year;
  uint32_t m = tm->month;
  if (m <= 2)
    y--;

  uint32_t era = y / 400;
  uint32_t yoe = y - era * 400;
  uint32_t doy = (153UL * (m > 2 ? m - 3 : m + 9) + 2) / 5 + tm->day - 1;
  uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  uint32_t days = era * 146097UL + doe - 719468UL;

  return days * 86400UL + tm->hour * 3600UL + tm->min * 60UL + tm->sec;
}

void meter_epoch_to_time(uint32_t epoch, ds1307_time_t *tm) {
  uint32_t days = epoch / 86400UL;
  uint32_t s = epoch % 86400UL;

  uint32_t z = days + 719468UL;
  uint32_t era = z / 146097UL;
  uint32_t doe = z - era * 146097UL;
  uint32_t yoe = (doe - doe / 1460 + doe / 36524UL - doe / 146096UL) / 365;
  uint32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  uint32_t mp = (5 * doy + 2) / 153;
  uint32_t m = mp < 10 ? mp + 3 : mp - 9;

  tm->day   = doy - (153 * mp + 2) / 5 + 1;
  tm->month = m;
  tm->year  = yoe + era * 400 + (m <= 2);
  tm->wday  = (days + 3) % 7 + 1; // 1 = Monday
  tm->hour  = s / 3600;
  tm->min   = (s % 3600) / 60;
  tm->sec   = s % 60;
}

//------------------------------------------------------------------------------

static inline bool meter_epoch_is_valid(uint32_t epoch) {
  return (epoch >= s_ram.t0);
}

static inline uint32_t meter_bucket_to_epoch(uint32_t bucket) {
  return s_ram.t0 + bucket * METER_WINDOW_SEC;
}

static inline uint32_t meter_epoch_to_bucket(uint32_t epoch) {
  return (epoch - s_ram.t0) / METER_WINDOW_SEC;
}

//------------------------------------------------------------------------------

// Bucket of the open window
static inline uint32_t meter_get_open_bucket(void) {
  uint32_t start = meter_blk_get_start(&s_ram.blk);
  return start + s_ram.blk.num;
}

static int32_t meter_get_now_bucket(void) {
  ds1307_time_t tm;
  if (!ds1307_get_time(&tm))
    return -1;

  uint32_t epoch = meter_time_to_epoch(&tm);
  if (!meter_epoch_is_valid(epoch))
    return -1;

  return (int32_t)meter_epoch_to_bucket(epoch);
}

//------------------------------------------------------------------------------

static void meter_blk_to_page(const meter_blk_t *blk, meter_page_t *page) {
  page->start = meter_blk_get_start(blk);
  page->num = blk->num;

  uint8_t i;
  for (i = 0; i < blk->num; i++)
    page->cnt[i] = meter_get_cnt(blk->data, i);

  for (; i < METER_PAGE_WINDOWS; i++)
    page->cnt[i] = METER_COUNT_UNKNOWN;
}

bool meter_read_page(uint8_t idx, meter_page_t *page) {
  if (idx >= METER_PAGES)
    return false;

  meter_blk_t blk;
  if (!eeprom_read(idx * EEPROM_PAGE_SIZE, (uint8_t *)&blk, sizeof(blk)))
    return false;

  if (!meter_blk_is_valid(&blk))
    return false;

  meter_blk_to_page(&blk, page);
  return true;
}

//------------------------------------------------------------------------------

// Write cur, cur_chk, blk to the DS1307 RAM in one transaction.
static bool meter_ram_save(void) {
  s_ram.blk.crc = meter_blk_calc_crc(&s_ram.blk);

  return ds1307_ram_write(METER_RAM_CUR_OFFSET,
      (const uint8_t *)&s_ram + METER_RAM_CUR_OFFSET,
      sizeof(meter_ram_t) - METER_RAM_CUR_OFFSET);
}

bool meter_write_eeprom(void) {
  if (!s_ram.blk.num)
    return true;

  s_ram.blk.crc = meter_blk_calc_crc(&s_ram.blk);

  uint16_t addr = (uint16_t)s_next_page * EEPROM_PAGE_SIZE;
  if (!eeprom_write(addr, (const uint8_t *)&s_ram.blk, sizeof(meter_blk_t)))
    return false;

  meter_blk_t check;
  if (!eeprom_read(addr, (uint8_t *)&check, sizeof(meter_blk_t)))
    return false;

  if (memcmp(&check, &s_ram.blk, sizeof(meter_blk_t)) != 0)
    return false;

  s_next_page = (s_next_page + 1) % METER_PAGES;
  return true;
}

// Find the newest valid page (highest start) -> next page to write.
static void meter_scan_eeprom(void) {
  uint32_t best = 0;
  s_next_page = 0;

  for (uint8_t i = 0; i < METER_PAGES; i++) {
    meter_blk_t blk;
    if (!eeprom_read(i * EEPROM_PAGE_SIZE, (uint8_t *)&blk, sizeof(meter_blk_t)))
      continue;

    if (!meter_blk_is_valid(&blk))
      continue;

    uint32_t start = meter_blk_get_start(&blk);
    if (best <= start) {  // >= : a duplicate page wins by its position
      best = start;
      s_next_page = (i + 1) % METER_PAGES;
    }
  }
}

//------------------------------------------------------------------------------

// Start an empty buffer whose first window is `bucket`.
static void meter_restart(uint32_t bucket) {
  memset(&s_ram.blk, 0xFF, sizeof(s_ram.blk));
  meter_blk_set_start(&s_ram.blk, bucket);

  s_ram.blk.num = 0;
  meter_set_cur(0);
  meter_swap_cnt(0);
  meter_ram_save();

  meter_set_flag_running();
}

// Close the open window with `cnt` pulses; the next window starts right away.
static void meter_close(uint16_t cnt) {
  if (cnt > METER_COUNT_MAX)
    cnt = METER_COUNT_MAX;  // saturate, 0xFFF is reserved for "unknown"

  // Bucket of the open window (the one being closed)
  uint32_t bucket = meter_get_open_bucket();

  // Update s_ram first, then save it in a single transaction.
  meter_set_cnt(s_ram.blk.data, s_ram.blk.num, cnt);  // value goes to slot `num`
  s_ram.blk.num++;                                    // the window is now closed
  meter_set_cur(0);                                   // new open window starts at 0
  meter_ram_save();                                   // blk + cur persisted at once

  if (s_ram.blk.num == METER_PAGE_WINDOWS) {
    meter_write_eeprom();       // on failure the buffer is dropped
    meter_restart(bucket + 1);  // next bucket
  }
}

static void meter_recover(uint32_t bucket) {
  if (meter_blk_is_valid(&s_ram.blk)) {
    // MCU restarted inside the same window: continue with the saved count.
    if (meter_get_open_bucket() == bucket) {
      uint16_t cnt = s_ram.cur;
      if (s_ram.cur_chk != meter_calc_cur_chk(s_ram.cur))
        cnt = 0;

      meter_swap_cnt(cnt);
      meter_set_flag_running();
      return;
    }

    // Otherwise the open window is incomplete: keep closed windows only.
    meter_write_eeprom();
  }

  meter_restart(bucket);
}

bool meter_reset(const ds1307_time_t *tm) {
  uint32_t epoch = meter_time_to_epoch(tm);

  uint8_t page_empty[EEPROM_PAGE_SIZE];
  memset(page_empty, 0xFF, sizeof(page_empty));

  for (uint8_t i = 0; i < METER_PAGES; i++) {
    if (!eeprom_write(i * EEPROM_PAGE_SIZE, page_empty, sizeof(page_empty)))
      return false;
  }

  if (!ds1307_set_time(tm))
    return false;

  // Round down
  s_ram.t0 = epoch - epoch % METER_WINDOW_SEC;
  s_ram.t0_crc = crc8_calc(METER_CRC_SEED, (const uint8_t *)&s_ram.t0, sizeof(s_ram.t0));

  if (!ds1307_ram_write(0, (const uint8_t *)&s_ram, METER_RAM_HEADER_SIZE))
    return false;

  meter_set_flag_t0_valid();

  s_next_page = 0;
  meter_restart(0);
  return true;
}

//------------------------------------------------------------------------------

static inline void meter_hw_init(void) {
  // INT0 = PD2, input with pull-up, falling edge (S0 output pulls low)
  DDRD  &= ~(1 << PD2);
  PORTD |= (1 << PD2);
  EICRA  = (1 << ISC01);
  EIFR   = (1 << INTF0);
  EIMSK |= (1 << INT0);

  // Timer0: normal mode, clk/1024, overflow interrupt
  TCCR0A = 0;
  TCCR0B = (1 << CS02) | (1 << CS00);
  TIMSK0 |= (1 << TOIE0);
}

//------------------------------------------------------------------------------

bool meter_init(void) {
  i2c_init(100000UL);
  if (!ds1307_init() || !eeprom_init())
    return false;

  meter_hw_init();
  meter_scan_eeprom();

  if (!ds1307_ram_read(0, (uint8_t *)&s_ram, sizeof(s_ram)))
    return false;

  uint8_t crc = crc8_calc(METER_CRC_SEED, (const uint8_t *)&s_ram.t0, sizeof(s_ram.t0));
  if (crc == s_ram.t0_crc) {
    meter_set_flag_t0_valid();

    int32_t bucket = meter_get_now_bucket();
    if (bucket >= 0)
      meter_recover((uint32_t)bucket);
  }

  sei();
  return true;
}

void meter_poll(void) {
  if (!meter_flag_is_running())
    return;

  // Save the open window count after each pulse (3 B, no wear on RAM).
  uint16_t cnt;
  uint8_t flags;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    cnt = s_cnt;
    flags = s_flags;
    s_flags &= ~METER_FLAG_DIRTY;
  }

  if (meter_flag_is_dirty()) {
    meter_set_cur(cnt);

    const uint8_t *src = (const uint8_t *)&s_ram + METER_RAM_CUR_OFFSET;
    if (!ds1307_ram_write(METER_RAM_CUR_OFFSET, src, METER_RAM_CUR_SIZE))
      meter_set_flag_dirty();  // retry next time
  }

  if ((uint8_t)(s_tick - s_last_poll) < METER_POLL_TICKS)
    return;
  s_last_poll = s_tick;

  int32_t now = meter_get_now_bucket();
  if (now < 0)
    return;

  uint32_t open = meter_get_open_bucket();
  if ((uint32_t)now == open)
    return;

  if ((uint32_t)now == open + 1) {
    cnt = meter_swap_cnt(0);
    meter_close(cnt);
  } else {
    // Clock jump or a stalled loop: keep closed windows, restart at `now`.
    meter_write_eeprom();
    meter_restart((uint32_t)now);
  }
}

//------------------------------------------------------------------------------

bool meter_set_time(const ds1307_time_t *tm) {
  if (!meter_flag_is_t0_valid())
    return meter_reset(tm);

  uint32_t epoch = meter_time_to_epoch(tm);
  if (epoch < s_ram.t0)
    return false;

  meter_write_eeprom(); // closed windows belong to the old timeline

  if (!ds1307_set_time(tm))
    return false;

  uint32_t bucket = meter_epoch_to_bucket(epoch);
  meter_restart(bucket);
  return true;
}

//------------------------------------------------------------------------------

 void meter_read_buffer(meter_page_t *page) {
  meter_blk_to_page(&s_ram.blk, page);
}

 uint16_t meter_current(void) {
  uint16_t c;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    c = s_cnt;
  }
  return c;
}

 uint32_t meter_t0(void) {
  return s_ram.t0;
}

 uint8_t meter_next_page(void) {
  return s_next_page;
}
