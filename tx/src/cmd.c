#include <ctype.h>
#include <avr/pgmspace.h>
#include <string.h>
#include "cmd.h"
#include "ds1307.h"
#include "meter.h"
#include "uart.h"

//------------------------------------------------------------------------------

#define CMD_LINE_MAX  40 // "SETTIME YYYY-MM-DD HH:MM:SS" is 27 chars

static char s_line[CMD_LINE_MAX];
static uint8_t s_len;
static bool s_overflow;

//------------------------------------------------------------------------------

static void uart_put_err(const char *name_pgm) {
  uart_puts_pgm(PSTR("ERR "));
  uart_puts_pgm(name_pgm);
  uart_putc('\n');
}

// Read exactly `digits` decimal digits.
static bool cmd_num(const char **s, uint8_t digits, uint16_t *ret) {
  uint16_t val = 0;

  for (uint8_t i = 0; i < digits; i++) {
    char c = **s;
    if (c < '0' || c > '9')
      return false;

    val = val * 10 + (c - '0');
    (*s)++;
  }

  *ret = val;
  return true;
}

static bool cmd_sep(const char **s, char c) {
  if (**s != c)
    return false;

  (*s)++;
  return true;
}

// "YYYY-MM-DD HH:MM:SS", also fills wday and rejects e.g. 31 February.
static bool cmd_parse_time(const char *s, ds1307_time_t *t) {
  uint16_t y, mo, d, h, mi, se;

  bool ret = cmd_num(&s, 4, &y) && cmd_sep(&s, '-') &&
             cmd_num(&s, 2, &mo) && cmd_sep(&s, '-') &&
             cmd_num(&s, 2, &d) && cmd_sep(&s, ' ') &&
             cmd_num(&s, 2, &h) && cmd_sep(&s, ':') &&
             cmd_num(&s, 2, &mi) && cmd_sep(&s, ':') &&
             cmd_num(&s, 2, &se) && *s == '\0';
  if (!ret)
    return false;

  if (y < 2000 || y > 2099 || mo < 1 || mo > 12 || d < 1 || d > 31 ||
      h > 23 || mi > 59 || se > 59)
    return false;

  t->year = y;
  t->month = mo;
  t->day = d;
  t->hour = h;
  t->min = mi;
  t->sec = se;

  // Round trip: an invalid date normalizes to a different day/month.
  ds1307_time_t chk;
  meter_epoch_to_time(meter_time_to_epoch(t), &chk);

  if (chk.day != t->day || chk.month != t->month)
    return false;

  t->wday = chk.wday;
  return true;
}

static void cmd_put_time(const ds1307_time_t *t) {
  uart_put_u32(t->year);
  uart_putc('-');
  uart_put_dec2(t->month);
  uart_putc('-');
  uart_put_dec2(t->day);
  uart_putc(' ');
  uart_put_dec2(t->hour);
  uart_putc(':');
  uart_put_dec2(t->min);
  uart_putc(':');
  uart_put_dec2(t->sec);
}

// Returns the arguments when `line` starts with the command, else NULL.
static const char *cmd_match(const char *line, const char *name_pgm) {
  size_t n = strlen_P(name_pgm);
  if (strncmp_P(line, name_pgm, n) != 0)
    return NULL;

  if (line[n] == '\0')
    return line + n;
  if (line[n] == ' ')
    return line + n + 1;

  return NULL;
}

//------------------------------------------------------------------------------

static void cmd_settime(const char *args) {
  ds1307_time_t t;

  if (!cmd_parse_time(args, &t))
    uart_put_err(PSTR("time"));
  else if (!meter_has_t0())
    uart_put_err(PSTR("no T0"));
  else if (!meter_set_time(&t))
    uart_put_err(PSTR("rejected"));
  else
    uart_puts_pgm(PSTR("OK\n"));
}

static void cmd_init(const char *args) {
  ds1307_time_t t;

  if (!cmd_parse_time(args, &t))
    uart_put_err(PSTR("time"));
  else if (!meter_reset(&t))
    uart_put_err(PSTR("I2C"));
  else
    uart_puts_pgm(PSTR("OK\n"));
}

static void cmd_gettime(void) {
  ds1307_time_t t;
  if (!ds1307_get_time(&t)) {
    uart_put_err(PSTR("rtc"));
    return;
  }

  uart_puts_pgm(PSTR("OK "));
  cmd_put_time(&t);

  uart_puts_pgm(PSTR(" T0="));
  if (meter_has_t0()) {
    ds1307_time_t t0;
    meter_epoch_to_time(meter_t0(), &t0);
    cmd_put_time(&t0);
  } else
    uart_puts_pgm(PSTR("none"));

  if (meter_state() == METER_RUNNING)
    uart_puts_pgm(PSTR(" RUNNING"));
  else
    uart_puts_pgm(PSTR(" NO_TIME"));

  uart_puts_pgm(PSTR(" CUR="));
  uart_put_u32(meter_current());
  uart_putc('\n');
}

static void cmd_exec(void) {
  const char *args;

  if ((args = cmd_match(s_line, PSTR("SETTIME"))))
    cmd_settime(args);
  else if ((args = cmd_match(s_line, PSTR("INIT"))))
    cmd_init(args);
  else if (cmd_match(s_line, PSTR("GETTIME")))
    cmd_gettime();
  else
    uart_put_err(PSTR("unknown"));
}

//------------------------------------------------------------------------------

void cmd_poll(void) {
  uint8_t c;

  while (uart_getc(&c)) {
    if (c == '\r')
      continue;

    if (c != '\n') {
      if (s_len < CMD_LINE_MAX - 1)
        s_line[s_len++] = toupper(c);
      else
        s_overflow = true;
      continue;
    }

    s_line[s_len] = '\0';

    bool overflow = s_overflow;
    s_overflow = false;

    bool empty = (s_len == 0);
    s_len = 0;

    if (overflow)
      uart_puts_pgm(PSTR("ERR too long\n"));
    else if (!empty)
      cmd_exec();

    return; // let meter_poll() run between lines
  }
}
