#include <zephyr.h>
#include <stdio.h>
#include <stdint.h>

#include "global_defines.h"
#include "upload_shim.h"
#include "flash_core.h"

/**********************************************************
*                                                 STATICS *
**********************************************************/
static const char TAG[] = "UPLOAD_SHIM";

/**********************************************************
*                                          IMPLEMENTATION *
**********************************************************/

// this uploads  once page at a time
// returns 
// -> 0 for uploaded
// -> 1 problem uploading
int upload_chunk(){
  uint8_t * ptr = NULL;
  uint32_t chunk;
  int rc; 

  // get the oldest page in flash
  get_oldest_page();

  for(int i = 0; i < UPLOAD_CHUNKS_IN_PAGE; i++){
    rc = upload_incident_chunk(&ptr, &chunk);
    if (rc == CHUNK_VALID){
      if(0 == ble_dump_chunk(ptr, UPLOAD_SIZE_CHUNK)){
        // setting the last packet in a page,
        // will trigger a page erase since
        // it would indicate all packets are 
        // sent to AWS
        set_chunk_sent(chunk);
      } else {
        ESP_LOGE(TAG, "Failed BLE write!");
        return 1;
      }
    } else if (rc == PAGE_FINISHED_UPLOAD){
      return 0;
    } else if (rc == CHUNK_PROBLEM){
      ESP_LOGE(TAG, "Unknown error in flash layer!");
      ASSERT(0);
    }  
  }

  // should never get here
  // quench the compiler warning
  return 0;
}


#ifdef DEBUG_CONFIG_DUMP_TO_BLE 
// this dumps from a message queue to ble,
// it's only used during testing, this way
// we bypass flash

// usual: trace->flash->ble
// debug: trace->ble  
int upload_from_trace(){
  ESP_LOGI(TAG, "Entering upload shim!");
  flash_packet_t flash;
  while( 0 == get_from_trace_debug_buffer(&flash) ){
     if(0 == ble_dump_chunk(&flash, sizeof(flash_packet_t))){
       ESP_LOGI(TAG, "Uploaded chunk!");
     } else {
       ESP_LOGI(TAG, "Failed to upload chunk");
       return 1;
     }
  }
  return 0;
}
#endif 
