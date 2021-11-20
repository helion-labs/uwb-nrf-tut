#pragma once

#include <stdint.h>
#include "ble_core.h"

/**********************************************************
*                                                 DEFINES *
**********************************************************/
#define STACK_SIZE_TRACE_THREAD    (1024)
#define TRACE_PRIORITY             (5)
#define MESSAGE_QUEUE_DEPTH        (4)    // size sensitive.. careful here
#define DEBUG_QUEUE_DEPTH          (MESSAGE_QUEUE_DEPTH * 20)
#define TRACE_PACKET_BUFFER_SIZE   (2)   // size sensitive.. also careful here
#define TRACE_LOOP_TIME            (720)
#define TIME_BEFORE_DUMP_IN_SECS   (60)  
#define DISTANCE_CM_SENTINIL_VAL   (0xFFFF)

/**********************************************************
*                                                   TYPES *
**********************************************************/


/**********************************************************
*                                                   TYPES *
**********************************************************/
// adv_packet_t gets filtered out, those that are from 
// other devices of interest become adv_packet_filtered_t
// devices of interest are chosen based on their name
typedef struct {
    uint8_t  valid;
    uint8_t  counts;
    int8_t   RSSI;
    uint32_t utc;
    uint8_t  manufactuers_data[BLE_MANUFACTURERS_DATA_LEN];
    uint16_t distance_cm;
}__attribute__((packed)) trace_packet_t;


/**********************************************************
*                                                 GLOBALS *
**********************************************************/
void spawn_trace_thread();
void init_trace_core();
void submit_adv_packet();
