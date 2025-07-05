#ifndef SPI_ASYNC
#define SPI_ASYNC
#include <stdint.h>
#include <stdbool.h>
#include "spi.h"

typedef void (*spi_callback_t)(void);

void spi_async_init(const SPIConfig* inst);
void spi_async_start_transfer(const SPIConfig* inst, const uint8_t *tx_buf, uint8_t *rx_buf, uint16_t length, spi_callback_t callback);
void spi_async_change_mode(const SPIConfig* inst);
bool spi_async_is_busy();

#endif

