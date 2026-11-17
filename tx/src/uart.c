#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "uart.h"

//------------------------------------------------------------------------------

#define UART_RX_SIZE  64 // power of 2

static volatile uint8_t s_rx[UART_RX_SIZE];
static volatile uint8_t s_head; // written by the ISR
static volatile uint8_t s_tail; // written by uart_getc()

//------------------------------------------------------------------------------

ISR(USART_RX_vect) {
  uint8_t st = UCSR0A;
  uint8_t c = UDR0;
  if (st & ((1 << FE0) | (1 << DOR0) | (1 << UPE0)))
    return;

  uint8_t next = (s_head + 1) & (UART_RX_SIZE - 1);
  if (next == s_tail)
    return; // full, drop the byte

  s_rx[s_head] = c;
  s_head = next;
}

//------------------------------------------------------------------------------

void uart_init(uint32_t baud) {
  UCSR0A = (1 << U2X0);
  UBRR0 = (uint16_t)((F_CPU + 4UL * baud) / (8UL * baud) - 1);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
  UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
}

bool uart_getc(uint8_t *c) {
  if (s_head == s_tail)
    return false;

  *c = s_rx[s_tail];
  s_tail = (s_tail + 1) & (UART_RX_SIZE - 1);
  return true;
}

void uart_putc(char c) {
  while (!(UCSR0A & (1 << UDRE0)))
    /* nop */;
  UDR0 = c;
}

void uart_puts_pgm(const char *s) {
  char c;
  while ((c = pgm_read_byte(s++)) != '\0')
    uart_putc(c);
}

void uart_put_u32(uint32_t val) {
  char buf[10];
  uint8_t n = 0;

  do {
    buf[n++] = '0' + val % 10;
    val /= 10;
  } while (val);

  while (n)
    uart_putc(buf[--n]);
}

void uart_put_dec(uint8_t val) {
  uart_putc('0' + val / 10);
  uart_putc('0' + val % 10);
}
