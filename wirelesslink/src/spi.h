#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>


void spi_init(void);
void spi_transmit(uint8_t* tx_buf, uint8_t* rx_buf, uint8_t len);


#endif

