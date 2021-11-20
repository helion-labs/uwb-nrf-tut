#include <zephyr.h>
#include <drivers/flash.h>
#include <storage/flash_map.h>
#include <device.h>
#include <stdio.h>
#include <string.h>

#include "ble_core.h"
#include "global_defines.h"
#include "flash_core.h"
#include "time_core.h"
#include "trace_core.h"

/**********************************************************
*                                                 STATICS *
**********************************************************/
static const char TAG[] = "UWB_CORE";

/**********************************************************
*                                                FORWARDS *
**********************************************************/

/**********************************************************
*                                          IMPLEMENTATION *
**********************************************************/
int uwb_scan_adv(){
  ESP_LOGI(TAG, "Starting UWB service!");
  set_red_led(1);

  int distance_cm;
    if (get_config_data() == 0xdead){
      distance_cm = main_resp();
       if (distance_cm > 0){
        ESP_LOGI(TAG, "Found peer, distance cm %d", distance_cm);
      }
    } else {
      distance_cm = main_init();        
      if (distance_cm > 0){
        ESP_LOGI(TAG, "Found peer, distance cm %d", distance_cm);
      }
    }
  ESP_LOGI(TAG, "Finished UWB service!");
  set_red_led(0);

  return distance_cm;
}
