#pragma once

#include <stdint.h>

/**********************************************************
*                                                 DEFINES *
**********************************************************/
#define STACK_SIZE_MASTER (2048)

/**********************************************************
*                                                   TYPES *
**********************************************************/
typedef enum {
    master_state_idle = 0,
    master_state_sntp,
    master_state_dump,
    
    master_state_len
} master_state_e;

typedef enum {
    MASTER_EVENT_SNTP = 0,
    MASTER_EVENT_DUMP,

    master_event_len //LEAVE AS LAST!
} master_event_e;

/**********************************************************
*                                                 GLOBALS *
**********************************************************/
void init_master_core();
