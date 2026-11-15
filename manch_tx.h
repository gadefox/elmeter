#ifndef MANCHESTER_TX_H
#define MANCHESTER_TX_H

#include <stdbool.h>
#include <stdint.h>

//------------------------------------------------------------------------------

void manch_tx_init(uint8_t pindata);
bool manch_tx_send(const uint8_t *data, uint8_t len);

#endif // MANCHESTER_TX_H
