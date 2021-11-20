#pragma once

#include <stdint.h>

/**********************************************************
*                                                 DEFINES *
**********************************************************/
#define LED_OFF (0)
#define LED_ON  (1)

#define CS_LOW  (0)
#define CS_HIGH (1)

#define V_BUS_DETECT (14)
#define RED_PIN      (13)
#define BLUE_PIN     (12)
#define SPI_CS       (8)
#define DECA_RST_PIN (10)
#define BUZZER_PIN   (27)

/**********************************************************
*                                                   TYPES *
**********************************************************/

/**********************************************************
*                                                 GLOBALS *
**********************************************************/
void init_gpio_core();
void set_red_led(bool);
void set_blue_led(bool);
bool is_charging();
void spi_set_cs_deca(bool);
void test_buzzer();
void reset_DW1000();
