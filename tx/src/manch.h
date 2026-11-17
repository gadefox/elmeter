#ifndef MANCH_TX_H
#define MANCH_TX_H

#include <stdbool.h>
#include <stdint.h>

//------------------------------------------------------------------------------

#define MANCH_MAX_PAYLOAD  32

//------------------------------------------------------------------------------

void manch_tx_init(void);
bool manch_tx_send(const uint8_t *data, uint8_t len);

#endif // MANCH_TX_H
