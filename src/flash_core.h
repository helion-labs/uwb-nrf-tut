#pragma once

#include <stdint.h>
#include "trace_core.h"
#include "ble_core.h"

/**********************************************************
*                                                 DEFINES *
**********************************************************/
#define FLASH_PAGE_SIZE                (4096)
#define SPARE_FLASH_PAGES_AT_END       (3)
#define IMAGE_SIZE_IN_PAGES            (0x3A) 
#define OFFSET_TO_FIRST_INCIDENCE_DATA (FLASH_AREA_OFFSET(image_1))  //this is in kb
#define TOTAL_PAGES_FOR_INCIDENCE_DATA (IMAGE_SIZE_IN_PAGES - SPARE_FLASH_PAGES_AT_END)
#define FLASH_SIZE_PACKET              (32)
#define FIRST_PAGE_INCIDENCE_DATA      (FLASH_AREA_OFFSET(image_1)/FLASH_PAGE_SIZE) //page numbers 
#define PACKETS_IN_PAGE                (FLASH_PAGE_SIZE / FLASH_SIZE_PACKET)
#define HEADER_PACKET_OFFSET           (0)

#define MAX_VALID_PAGE            (127)
#define FINAL_VALID_PAGE          (127-SPARE_FLASH_PAGES_AT_END) //last page is for misc use
#define FINAL_VALID_PACKET_OFFSET (PACKETS_IN_PAGE - 1)

#define PAGE_HEADER_MAGIC       (0xDEAD)
#define PAGE_NORMAL_ENTRY_MAGIC (0xC0FE)
#define PAGE_HEADER_MAGIC_EMTPY (0xFFFF)

#define MAX_VALID_ID (0xFFFFFFFF)

#define FLASH_PACKETS_PER_CHUNK (8)
#define UPLOAD_SIZE_CHUNK       (FLASH_SIZE_PACKET * FLASH_PACKETS_PER_CHUNK) 
#define UPLOAD_CHUNKS_IN_PAGE   (FLASH_PAGE_SIZE / UPLOAD_SIZE_CHUNK ) 

#define CHUNK_VALID          (0)
#define PAGE_FINISHED_UPLOAD (1)
#define CHUNK_PROBLEM        (2)
#define UPLOADING_DONE       (3)

//#define TEST_MODE_FLASH // if set, will do a quick sanity check of flash

#define DEVICE_BLE (0xDEAD)

/**********************************************************
*                                                   TYPES *
**********************************************************/
// keep this in sync with what the ESP thinks of as a flash packet
// This is the format we save to flash
// keep it exactly 32 bytes
typedef struct {
    uint16_t type;
    union {
      uint32_t id;           // Only valid for type == page header
      uint32_t distance_uwb; // Only valid for readings
    } specifics;
    uint8_t  manufactuers_data[BLE_MANUFACTURERS_DATA_LEN];
    int8_t   RSSI;
    uint8_t  counts;
    int32_t  utc;
}__attribute__((packed)) flash_packet_t;
_Static_assert(sizeof(flash_packet_t) == FLASH_SIZE_PACKET, "flash packet is not 32 bytes long!");

typedef struct {
  uint32_t oldest_id;
  uint32_t oldest_id_page;
  uint32_t olest_id_age_utc;
} flash_oldest_t;

typedef struct {
  uint32_t current_id;
  uint16_t current_valid_page;
  uint16_t current_valid_packet_in_page;
  uint16_t total_valid_pages;
} flash_curr_t;

/**********************************************************
*                                                 GLOBALS *
**********************************************************/
void     init_flash_core();
uint32_t get_oldest_page();
bool     get_flash_full();
void     write_packet(trace_packet_t* trace_packet);
int      upload_incident_chunk(uint8_t ** data, uint32_t * ready_chunk);
void     set_chunk_sent(uint32_t chunk);
void     take_flash_sem();
void     give_flash_sem();
bool     flash_is_empty();
uint32_t get_age_oldest_page();
uint16_t get_total_valid_pages();
void     set_config_data(int id);
int      get_config_data();
