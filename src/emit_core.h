#pragma once

#include <zephyr.h>
#include <stdint.h>

/**********************************************************
*                                                 DEFINES *
**********************************************************/
#define STACK_SIZE_EMIT                          (1024)
#define WAIT_PERIOD_SNTP                         (60)
#define WAIT_PERIOD_DUMP                         (60*10) //Might take a while to unload
#define SLEEP_PERIOD                             (17)    //random prime number
#define MAX_AGE_PARTIAL_PAGE_BEFORE_UPLOAD_HOURS (4)

/**********************************************************
*                                                   TYPES *
**********************************************************/
typedef enum {
    emit_state_check_status,
    emit_state_scan_advertise,
    emit_state_fota,     //this state also dumps the flash

    emit_state_len
} emit_state_e;

// Emit core does not react to any events,
// it's driven by the state returns 
typedef enum {
    EMIT_EVENT_NONE = 0,         

    emit_event_len //LEAVE AS LAST!
} emit_event_e;

/**********************************************************
*                                                 GLOBALS *
**********************************************************/
void init_emit_core();
