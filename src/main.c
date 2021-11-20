/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <sys/util.h>
#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>
#include <bluetooth/scan.h>

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>

#include "trace_core.h"
#include "flash_core.h"
#include "time_core.h"
#include "gpio_core.h"
#include "ble_core.h"
#include "signals_core.h"
#include "master_core.h"
#include "emit_core.h"
#include "fota_core.h"
#include "spi_core.h"
#include "global_defines.h"
#include "uwb_core.h"

/**********************************************************
*                                                 STATICS *
**********************************************************/
static const char TAG[] = "MAIN";


void main(void) {
  // get the mac, this way we decide if we are 
  k_sleep(K_MSEC(1000)); // allow time for the prints to come

  init_gpio_core(); 
  spi_init_core();
  init_flash_core();
  init_ble_core();
  init_signals_core();
 
  ESP_LOGI(TAG, "Device specifc data = %d", get_config_data());

  k_sleep(K_MSEC(1000));

  // connect to a central device bia BLE
  connect_core_device();

  // From SNTP, get the network time
  k_sleep(K_MSEC(2000)); // niave, should check if we actually connected to BLE - but
                         // for this tutorial we will assume after 2 minutes we are connected

  // get the the from NTP characteristic on the core device
  get_nw_time();

  // Only one device will run the BLE code
  if (get_config_data() == DEVICE_BLE ){
    while(true){
      k_sleep(K_MSEC(1000));
      int distance = uwb_scan_adv();

      if ( distance <= 0 ){
        continue;
      }

      uwb_packet_t uwb;
      uwb.distance_uwb = distance;
      uwb.time         = get_current_time();

      ESP_LOGI(TAG, "Distance = %d, Time = %d", distance, uwb.time);
      ble_dump_chunk(&uwb, sizeof(uwb_packet_t));
    }
  } else {
     // This device will not use BLE to send 
     // Data to the cloud, the other device will do
     // this
     while(true){
      k_sleep(K_MSEC(1000));
      uwb_scan_adv();
     }
  }
}
