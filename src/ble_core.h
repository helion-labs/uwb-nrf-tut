#pragma once

#include <stdint.h>

/**********************************************************
*                                                 DEFINES *
**********************************************************/
// Advertising
#define MAX_USER_DATA_LEN       (100)
#define DEVICE_NAME             ("CW_100_EDGE")
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

//UIDS
#define BT_UUID_SERVICE_CW    BT_UUID_DECLARE_16(0x00FF)  // Service that contains the follwoing..
#define BT_UUID_TIME_VALUE    BT_UUID_DECLARE_16(0xB0F0)  // read internet time on ESP32 (SNTP)
#define BT_UUID_DUMP_VALUE    BT_UUID_DECLARE_16(0xDEAD)  // place to dump incidences to 
#define BT_UUID_FW_VER_VALUE  BT_UUID_DECLARE_16(0xC0FE)  // lastest FW version
#define BT_UUID_FW_REQ_VALUE  BT_UUID_DECLARE_16(0xFACE)  // request a fw chunk
#define BT_UUID_FW_VALUE      BT_UUID_DECLARE_16(0x1234)  // actual FW chunk

//Scan
#define SCAN_DURATION_SECONDS (15)

//Misc
#define MAX_USER_DATA_LEN          (100)
#define BLE_MANUFACTURERS_DATA_LEN (20)
#define BLE_MANUFACTURERS_NAME_LEN (20)

// this is the ESP32 device 
#define CORE_DEVICE_NAME           ("CW_100_CORE")
#define CORE_DEVICE_LEN            (strlen(CORE_DEVICE_NAME))
#define ADV_SIZE_PACKET            (64) 

//Timeouts
#define CONNECTION_TIMEOUT (30) // how long to wait for a connection
#define DISCONNECT_TIMEOUT (30) // how long to wait for a disconnection
#define WRITE_TIMEOUT      (60) // how long to wait for a disconnection
#define SNTP_TIMEOUT       (30) // how long to wait for GATT read of SNTP

// GATT reads
#define SINGLE_READ (1)

// If defined, makes fake incidence readings
// #define BLE_TEST_MODE 
/**********************************************************
*                                                   TYPES *
**********************************************************/
// to do, move this someplace else, it's used in UWB as well
typedef struct {
    uint8_t  name_complete[BLE_MANUFACTURERS_NAME_LEN]; 
    uint8_t  manufactuers_data[BLE_MANUFACTURERS_DATA_LEN];
    int8_t   RSSI;
    uint16_t uwb_distance_cm;
    uint8_t spare[21];
}__attribute__((packed)) adv_packet_t;
_Static_assert(sizeof(adv_packet_t) == ADV_SIZE_PACKET, "adv packet is not 64 bytes long! (needs to be 2^n allgined)");


// a key to the handle to UUIDs of interest
typedef struct {
  bool     valid;
  uint16_t time;
  uint16_t dump;
  uint16_t fw_ver;
  uint16_t fw_req;
  uint16_t fw_val;
} handle_key_t;

/**********************************************************
*                                                 GLOBALS *
**********************************************************/
void init_ble_core();
int  connect_core_device();
int  get_nw_time();
int  start_scan_record();
void connection_cleanup();
int  ble_dump_chunk(uint8_t * chunk, size_t len);
