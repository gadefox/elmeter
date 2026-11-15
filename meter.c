#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>
#include <string.h>
#include <util/atomic.h>
#include "i2c.h"
#include "ds1307.h"
#include "eeprom.h"
#include "meter.h"

//------------------------------------------------------------------------------

#define METER_DEBOUNCE_MS  30UL
#define METER_POLL_MS      250UL

// Timer0 overflow tick, prescaler 1024 (16384 us at 16 MHz).
#define METER_TICK_US  (262144000UL / (F_CPU / 1000UL))

#define METER_DEBOUNCE_TICKS \
  ((uint8_t)((METER_DEBOUNCE_MS * 1000UL + METER_TICK_US - 1) / METER_TICK_US + 1))
#define METER_POLL_TICKS \
  ((uint8_t)(METER_POLL_MS * 1000UL / METER_TICK_US))

#define METER_DATA_BYTES  (METER_PAGE_WINDOWS * 3 / 2)
#define METER_CRC_SEED    0xFF
#define METER_CUR_SEED    0x5A

//------------------------------------------------------------------------------

// One block = one EEPROM page = image of the closed windows in DS1307 RAM.
typedef struct {
  uint8_t start[3];                   // bucket of the first window (LE)
  uint8_t n;                          // number of closed windows
  uint8_t data[METER_DATA_BYTES];     // 12 bit values, unused bytes 0xFF
  uint8_t crc;                        // CRC-8 over everything above
} __attribute__((packed)) meter_blk_t;

// Layout of the DS1307 RAM (offset 0..39).
typedef struct {
  uint32_t    t0;                     // epoch of window 0, set via UART
  uint8_t     t0_crc;
  meter_blk_t blk;
  uint16_t    cur;                    // open window, last saved value
  uint8_t     cur_chk;
} __attribute__((packed)) meter_ram_t;

_Static_assert(sizeof(meter_blk_t) == EEPROM_PAGE_SIZE, "blk != EEPROM page");
_Static_assert(sizeof(meter_ram_t) <= DS1307_RAM_SIZE, "RAM layout too big");
_Static_assert(METER_PAGES == EEPROM_SIZE / EEPROM_PAGE_SIZE, "page count");

#define METER_COMMIT_OFFSET  offsetof(meter_ram_t, blk)
#define METER_COMMIT_LEN     (sizeof(meter_ram_t) - METER_COMMIT_OFFSET)

//------------------------------------------------------------------------------

static meter_ram_t s_ram;
static meter_state_t s_state = METER_NO_TIME;
static bool s_t0_ok;
static uint8_t s_next_page;

static volatile uint8_t s_tick;
static volatile uint16_t s_cnt;
static volatile bool s_dirty;
static uint8_t s_last_pulse;
static uint8_t s_last_poll;

//------------------------------------------------------------------------------

ISR(TIMER0_OVF_vect) {
  g_tick++;
}

ISR(INT0_vect) {
  uint8_t now = g_tick;
  if ((uint8_t)(now - g_last_pulse) < METER_DEBOUNCE_TICKS)
    return;

  g_last_pulse = now;
  g_cnt++;
  g_dirty = true;
}

//------------------------------------------------------------------------------

static uint8_t meter_crc8(const uint8_t *p, uint8_t n) {
  uint8_t crc = METER_CRC_SEED;
  while (n--) {
    crc ^= *p++;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

static inline uint8_t meter_blk_crc(const meter_blk_t *blk) {
  return meter_crc8((const uint8_t *)blk, sizeof(meter_blk_t) - 1);
}

static bool meter_blk_is_valid(const meter_blk_t *blk) {
  return blk->num <= METER_PAGE_WINDOWS && blk->crc == meter_blk_crc(blk);
}

static uint32_t meter_blk_start(const meter_blk_t *blk) {
  return (uint32_t)blk->start[0] | ((uint32_t)blk->start[1] << 8) |
         ((uint32_t)blk->start[2] << 16);
}

static void meter_blk_set_start(meter_blk_t *blk, uint32_t val) {
  blk->start[0] = val & 0xFF;
  blk->start[1] = (val >> 8) & 0xFF;
  blk->start[2] = (val >> 16) & 0xFF;
}

static inline uint32_t meter_open_bucket(void) {
  return meter_blk_start(&s_ram.blk) + s_ram.blk.num;
}

static void meter_put12(uint8_t *d, uint8_t i, uint16_t v) {
  uint8_t *p = d + (i >> 1) * 3;
  if (i & 1) {
    p[1] = (p[1] & 0xF0) | (uint8_t)(v >> 8);
    p[2] = v & 0xFF;
  } else {
    p[0] = (uint8_t)(v >> 4);
    p[1] = (p[1] & 0x0F) | (uint8_t)((v & 0x0F) << 4);
  }
}

static uint16_t meter_get12(const uint8_t *d, uint8_t i) {
  const uint8_t *p = d + (i >> 1) * 3;
  if (i & 1)
    return ((uint16_t)(p[1] & 0x0F) << 8) | p[2];

  return ((uint16_t)p[0] << 4) | (p[1] >> 4);
}

static inline uint8_t meter_cur_chk(uint16_t cur) {
  return (cur & 0xFF) ^ (cur >> 8) ^ METER_CUR_SEED;
}

static inline void meter_set_cur(uint16_t cur) {
  s_ram.cur = cur;
  s_ram.cur_chk = meter_cur_chk(cur);
}

static void meter_decode(const meter_blk_t *blk, meter_page_t *page) {
  page->start = meter_blk_start(blk);
  page->n = blk->n;

  for (uint8_t i = 0; i < METER_PAGE_WINDOWS; i++)
    page->cnt[i] = i < blk->n ? meter_get12(blk->data, i) : METER_COUNT_UNKNOWN;
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

static void meter_epoch_to_time(uint32_t epoch, ds1307_time_t *tm) {
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

static inline uint32_t meter_bucket_to_epoch(uint32_t bucket) {
  return s_ram.t0 + bucket * METER_WINDOW_SEC;
}

static bool meter_now_bucket(uint32_t *bucket) {
  ds1307_time_t tm;
  if (!ds1307_get_time(&tm))
    return false;

  uint32_t epoch = meter_time_to_epoch(&tm);
  if (epoch < s_ram.t0)
    return false;

  *bucket = (epoch - s_ram.t0) / METER_WINDOW_SEC;
  return true;
}

//------------------------------------------------------------------------------

// Write blk + cur to the DS1307 RAM in one transaction.
static bool meter_commit(void) {
  s_ram.blk.crc = meter_blk_crc(&s_ram.blk);
  return ds1307_ram_write(METER_COMMIT_OFFSET,
                          (const uint8_t *)&s_ram + METER_COMMIT_OFFSET,
                          METER_COMMIT_LEN);
}

// Start an empty buffer whose first window is `bucket`.
static void meter_restart(uint32_t bucket) {
  memset(&s_ram.blk, 0xFF, sizeof(s_ram.blk));
  meter_blk_set_start(&s_ram.blk, bucket);
  s_ram.blk.num = 0;
  meter_set_cur(0);

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    s_cnt = 0;
    s_dirty = false;
  }
  meter_commit();
}

bool meter_flush(void) {
  if (s_state != METER_RUNNING || s_ram.blk.num == 0)
    return true;

  s_ram.blk.crc = meter_blk_crc(&s_ram.blk);

  uint16_t addr = (uint16_t)s_next_page * EEPROM_PAGE_SIZE;

  meter_blk_t chk;
  if (!eeprom_write(addr, (const uint8_t *)&s_ram.blk, sizeof(chk)))
    return false;
  if (!eeprom_read(addr, (uint8_t *)&chk, sizeof(chk)))
    return false;
  if (memcmp(&chk, &s_ram.blk, sizeof(chk)) != 0)
    return false;

  s_next_page = (s_next_page + 1) % METER_PAGES;
  return true;
}

// Close the open window with `cnt` pulses; the next window starts right away.
static void meter_close(uint16_t cnt) {
  uint32_t next = meter_open_bucket() + 1;
  if (cnt > METER_COUNT_MAX)
    cnt = METER_COUNT_MAX;

  meter_put12(s_ram.blk.data, s_ram.blk.num, cnt);
  s_ram.blk.num++;
  meter_set_cur(0);
  meter_commit();

  if (s_ram.blk.num >= METER_PAGE_WINDOWS) {
    meter_flush();          // on failure the buffer is dropped
    meter_restart(next);
  }
}

//------------------------------------------------------------------------------

// Find the newest valid page (highest start) -> next page to write.
static void meter_scan(void) {
  meter_blk_t blk;
  uint32_t best = 0;
  bool found = false;

  s_next_page = 0;
  for (uint8_t i = 0; i < METER_PAGES; i++) {
    if (!eeprom_read(i * EEPROM_PAGE_SIZE, (uint8_t *)&blk, sizeof(blk)))
      continue;
    if (!meter_blk_is_valid(&blk))
      continue;

    uint32_t start = meter_blk_start(&b);
    if (!found || start >= best) {  // >= : a duplicate page wins by its position
      best = start;
      found = true;
      s_next_page = (i + 1) % METER_PAGES;
    }
  }
}

static void meter_recover(uint32_t now) {
  bool ret = meter_blk_is_valid(&g_ram.blk);

  // MCU restarted inside the same window: continue with the saved count.
  if (ret && meter_open_bucket() == now) {
    uint16_t c = s_ram.cur;
    if (s_ram.cur_chk != meter_cur_chk(c))
      c = 0;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      s_cnt = c;
      s_dirty = false;
    }
    return;
  }

  // Otherwise the open window is incomplete: keep closed windows only.
  s_state = METER_RUNNING; // allow meter_flush()
  if (ret)
    meter_flush();

  meter_restart(now);
}

static void meter_hw_init(void) {
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

  s_last_pulse = (uint8_t)(s_tick - METER_DEBOUNCE_TICKS);
  s_last_poll = s_tick;
}

//------------------------------------------------------------------------------

bool meter_init(void) {
  i2c_init(100000UL);
  if (!ds1307_init() || !eeprom_init())
    return false;

  meter_hw_init();
  meter_scan();

  if (!ds1307_ram_read(0, (uint8_t *)&s_ram, sizeof(s_ram)))
    return false;

  s_t0_ok = s_ram.t0_crc == meter_crc8((const uint8_t *)&s_ram.t0,
                                       sizeof(s_ram.t0));
  s_state = METER_NO_TIME;

  uint32_t now;
  if (s_t0_ok && ds1307_is_running() && meter_now_bucket(&now)) {
    meter_recover(now);
    s_state = METER_RUNNING;
  }

  sei();
  return true;
}

void meter_poll(void) {
  if (s_state != METER_RUNNING)
    return;

  // Save the open window count after each pulse (3 B, no wear on RAM).
  uint16_t c;
  bool dirty;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    c = s_cnt;
    dirty = s_dirty;
    s_dirty = false;
  }

  if (dirty) {
    meter_set_cur(c);
    if (!ds1307_ram_write(offsetof(meter_ram_t, cur),
                          (const uint8_t *)&s_ram + offsetof(meter_ram_t, cur),
                          3))
      s_dirty = true; // retry next time
  }

  if ((uint8_t)(s_tick - s_last_poll) < METER_POLL_TICKS)
    return;
  s_last_poll = s_tick;

  uint32_t now;
  if (!meter_now_bucket(&now))
    return;

  uint32_t open = meter_open_bucket();
  if (now == open)
    return;

  if (now == open + 1) {
    uint16_t cnt;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      cnt = s_cnt;
      s_cnt = 0;
      s_dirty = false;
    }
    meter_close(cnt);
  } else {
    // Clock jump or a stalled loop: keep closed windows, restart at `now`.
    meter_flush();
    meter_restart(now);
  }
}

meter_state_t meter_get_state(void) {
  return s_state;
}

//------------------------------------------------------------------------------

bool meter_reset(const ds1307_time_t *tm) {
  uint32_t epoch = meter_time_to_epoch(tm);

  uint8_t buf[EEPROM_PAGE_SIZE];
  memset(buf, 0xFF, sizeof(buf));

  for (uint8_t i = 0; i < METER_PAGES; i++) {
    if (!eeprom_write(i * EEPROM_PAGE_SIZE, buf, sizeof(buf)))
      return false;
  }

  if (!ds1307_set_time(t))
    return false;

  s_ram.t0 = epoch - epoch % METER_WINDOW_SEC;
  s_ram.t0_crc = meter_crc8((const uint8_t *)&s_ram.t0, sizeof(s_ram.t0));

  if (!ds1307_ram_write(0, (const uint8_t *)&s_ram, 5))
    return false;

  s_t0_ok = true;
  s_next_page = 0;
  meter_restart(0);

  s_state = METER_RUNNING;
  return true;
}

bool meter_set_time(const ds1307_time_t *tm) {
  if (!s_t0_ok)
    return meter_reset(tm);

  uint32_t epoch = meter_time_to_epoch(t);
  if (epoch < s_ram.t0)
    return false;

  meter_flush(); // closed windows belong to the old timeline

  if (!ds1307_set_time(tm))
    return false;

  s_state = METER_RUNNING;
  meter_restart((epoch - s_ram.t0) / METER_WINDOW_SEC);
  return true;
}

//------------------------------------------------------------------------------

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

bool meter_read_page(uint8_t idx, meter_page_t *page) {
  if (idx >= METER_PAGES)
    return false;

  meter_blk_t blck;
  if (!eeprom_read(idx * EEPROM_PAGE_SIZE, (uint8_t *)&blck, sizeof(blck)))
    return false;

  if (!meter_blk_is_valid(&blck))
    return false;

  meter_decode(&blk, page);
  return true;
}

void meter_read_buffer(meter_page_t *page) {
  meter_decode(&s_ram.blk, page);
}
