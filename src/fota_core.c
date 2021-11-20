#include <zephyr.h>
#include <stdio.h>
#include <stdint.h>
#include <storage/flash_map.h>
#include <dfu/flash_img.h>
#include <dfu/mcuboot.h>
#include <storage/flash_map.h>

#include "global_defines.h"

/**********************************************************
*                                                 STATICS *
**********************************************************/
static const char TAG[] = "FOTA_CORE";

/**********************************************************
*                                                FORWARDS *
**********************************************************/


/**********************************************************
*                                          IMPLEMENTATION *
**********************************************************/
static int start_fw_update(){
  if(boot_request_upgrade(true) == 0){
    ESP_LOGI(TAG, "FW update started");
    return 0;
  } else {
    ESP_LOGI(TAG, "Failed to start FW update");
    return 1;
  }
  
  // should not get here, 
  // quench compiler warning
  return 1;
}

int switch_image(){
  struct mcuboot_img_header header;
  boot_read_bank_header(FLASH_AREA_ID(image_1), &header, sizeof(header));
  
  if (0 == start_fw_update()){
    k_sleep(K_SECONDS(5));
    sys_reboot(1);
    return 0;
  }
  ESP_LOGE(TAG, "Failed to swap image!");
  return 1;
}

// this is safe to call all teh time, we will do it 
// each boot because otherwise we won't know if we
// did a fota or not
void finalize_image(){
  boot_write_img_confirmed();
}
