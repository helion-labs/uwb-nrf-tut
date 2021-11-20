#include <zephyr.h>
#include <stdint.h>

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include "global_defines.h"
#include "gpio_core.h"

/**********************************************************
*                                                 STATICS *
**********************************************************/
static const struct device *dev;
static const char TAG[] = "GPIO_CORE";
/**********************************************************
*                                                 FORWARDS *
**********************************************************/
static int config_pins();

/**********************************************************
*                                          IMPLEMENTATION *
**********************************************************/

void init_gpio_core(){
  ESP_LOGE(TAG, "Initing GPIO core!\n");
  
  dev = device_get_binding("GPIO_0");
  if (dev == NULL) {
    ESP_LOGE(TAG, "Nordic nRF5 GPIO driver was not found!\n");
    ASSERT(0);
  }

  if(config_pins()){
    ESP_LOGE(TAG, "Failed to set up pins \n");
    ASSERT(0);
  }
}

static int config_pins(){
  int ret = gpio_pin_configure(dev, RED_PIN, GPIO_OUTPUT_ACTIVE);
  if (ret != 0) {
    ESP_LOGE(TAG, "failed to init red LED %d \n", ret);
    return 1;
  }

  ret = gpio_pin_configure(dev, BLUE_PIN, GPIO_OUTPUT_ACTIVE);
  if (ret != 0) {
    ESP_LOGE(TAG, "failed to init BLUE LED %d \n", ret);
    return 1;
  }
  
  ret = gpio_pin_configure(dev, V_BUS_DETECT, GPIO_INPUT);
  if (ret != 0) {
    ESP_LOGE(TAG, "failed to init GHARGE_V_BUS %d \n", ret);
    return 1;
  }
  
  // Set up the SPI pins
  ret = gpio_pin_configure(dev, SPI_CS, GPIO_OUTPUT_ACTIVE);
  if (ret != 0) {
    ESP_LOGE(TAG, "failed to init CS_SPI_DECA %d \n", ret);
    return 1;
  }

  return 0;
}

void set_red_led(bool on){
  if (!on){
    gpio_pin_set(dev, RED_PIN, LED_ON);
  } else {
    gpio_pin_set(dev, RED_PIN, LED_OFF);
  }
}

void set_blue_led(bool on){
  if (!on){
    gpio_pin_set(dev, BLUE_PIN, LED_ON);
  } else {
    gpio_pin_set(dev, BLUE_PIN, LED_OFF);
  }
}

bool is_charging(){
  return (bool)gpio_pin_get(dev, V_BUS_DETECT);
}

void spi_set_cs_deca(bool set){
  if(set){
    gpio_pin_set(dev, SPI_CS, CS_HIGH);
  } else {
    gpio_pin_set(dev, SPI_CS, CS_LOW);
  }
}

void reset_DW1000(){
  int ret = gpio_pin_configure(dev, DECA_RST_PIN, GPIO_OUTPUT_LOW);
  if (ret != 0) {
    ESP_LOGE(TAG, "failed to set deca-reset GPIO low! err = %d \n", ret);
  }
  
  k_sleep(K_MSEC(250));
  
  ret = gpio_pin_configure(dev, DECA_RST_PIN, GPIO_DISCONNECTED);
  if (ret != 0) {
    ESP_LOGE(TAG, "failed to set deca-reset GPIO high-z! err = %d \n", ret);
  }
}
