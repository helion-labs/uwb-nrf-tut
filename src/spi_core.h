#pragma once

#include <stdint.h>

/**********************************************************
*                                                 DEFINES *
**********************************************************/

/**********************************************************
*                                                   TYPES *
**********************************************************/

/**********************************************************
*                                                 GLOBALS *
**********************************************************/
void spi_init_core(void);
void spi_test_send(void);
int writetospi(uint16_t headerLength, uint8_t *headerBuffer, uint32_t bodyLength, uint8_t *bodyBuffer);
int readfromspi(uint16_t headerLength, uint8_t *headerBuffer, uint32_t readLength, uint8_t *readBuffer);
void port_set_dw1000_slowrate(void);
void port_set_dw1000_fastrate(void);
