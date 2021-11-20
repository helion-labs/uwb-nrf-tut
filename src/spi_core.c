#include <zephyr.h>
#include <stdio.h>
#include <stdint.h>
#include "global_defines.h"
#include "deca_device_api.h"

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>

#include "gpio_core.h"


/**********************************************************
*                                                 STATICS *
**********************************************************/
static const char TAG[] = "SPI_CORE";

static const struct spi_config spi_cfg_fast = {
  .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
  .frequency = 20000000,
  .slave = 0,
};

static const struct spi_config spi_cfg_slow = {
  .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
  .frequency = 3000000,
  .slave = 0,
};

static struct k_mutex  spi_mutex;
static struct device const * spi_dev;

// if set, use the slow spi
// else, use fast spi
static bool fast;

/**********************************************************
*                                                FORWARDS *
**********************************************************/

/**********************************************************
*                                          IMPLEMENTATION *
**********************************************************/
void spi_init_core(void){
  // set up the mutex 
  k_mutex_init(&spi_mutex);
  
  spi_dev = device_get_binding("SPI_0");

  if (spi_dev == NULL) {
    printf("Could not get device\n");
    return;
  }
}

int writetospi(uint16_t headerLength, uint8_t *headerBuffer, uint32_t bodyLength, uint8_t *bodyBuffer) {
    struct spi_buf_set tx;
    const struct spi_config * spi_cfg;
    
    k_mutex_lock(&spi_mutex, K_FOREVER);   
    if (fast){
      spi_cfg = &spi_cfg_fast;
    } else {
      spi_cfg = &spi_cfg_slow;
    }

    spi_set_cs_deca(CS_LOW);

    struct spi_buf tx_buffer[2];
    tx_buffer[0].buf     = headerBuffer;
    tx_buffer[0].len     = headerLength;
    tx_buffer[1].buf     = bodyBuffer;
    tx_buffer[1].len     = bodyLength;

    tx.buffers = tx_buffer;
    tx.count = 2;
    
    int status = spi_write(spi_dev, spi_cfg, &tx);
    if (status != 0){
      ESP_LOGE(TAG, "Failed to write to flash!");
    } 

    spi_set_cs_deca(CS_HIGH);
    k_mutex_unlock(&spi_mutex);
    return 0;
} 

int readfromspi(uint16_t headerLength, uint8_t *headerBuffer, uint32_t readLength, uint8_t *readBuffer) {
    struct spi_buf_set tx;
    struct spi_buf_set rx;
    struct spi_buf tx_buffer;
    struct spi_buf rx_buffer;
    const struct spi_config * spi_cfg;
    
    k_mutex_lock(&spi_mutex, K_FOREVER);   
    if (fast){
      spi_cfg = &spi_cfg_fast;
    } else {
      spi_cfg = &spi_cfg_slow;
    }
  
    //CS low
    spi_set_cs_deca(CS_LOW);
    tx_buffer.buf = headerBuffer;
    tx_buffer.len = headerLength;

    tx.buffers = &tx_buffer;
    tx.count   = 1;

    int status = spi_write(spi_dev, spi_cfg, &tx);
    if (status != 0){
      ESP_LOGE(TAG, "Failed to write to flash!");
    } 
    
    rx_buffer.buf = readBuffer;
    rx_buffer.len = readLength;
    rx.buffers = &rx_buffer;
    rx.count   = 1;

    status = spi_read(spi_dev, spi_cfg, &rx);
    if (status != 0){
      ESP_LOGE(TAG, "Failed to read body from flash!");
    } 
    k_mutex_unlock(&spi_mutex);
    
    spi_set_cs_deca(CS_HIGH);
    return 0;
} // end readfromspi()


decaIrqStatus_t decamutexon(void){
  return irq_lock();
}

void decamutexoff(decaIrqStatus_t stat){
  irq_unlock(stat);
}

void port_set_dw1000_slowrate(void){
  fast = false;
}

void port_set_dw1000_fastrate(void){
  fast = true;
}
