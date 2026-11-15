#ifndef UART_H
#define UART_H

#include <stdbool.h>
#include <stdint.h>

//------------------------------------------------------------------------------

// 8N1, RX interrupt + ring buffer, TX blocking. Needs global interrupts on.
void uart_init(uint32_t baud);

// Non-blocking read, false when the RX buffer is empty.
bool uart_getc(uint8_t *c);

void uart_putc(char c);
void uart_puts_pgm(const char *s);  // string in flash: uart_puts_P(PSTR("..."))
void uart_put_u32(uint32_t val);    // decimal
void uart_put_dec99(uint8_t val);   // 2 digits, zero padded

#endif // UART_H
