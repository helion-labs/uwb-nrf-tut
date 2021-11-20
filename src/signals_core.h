#pragma once

#include <stdint.h>

/**********************************************************
*                                                 DEFINES *
**********************************************************/
#define TOTAL_NUMBER_OF_EVENTS (1)
#define DONE_VAL (0)

// top level
#define WAIT_FOR_WORKER      (1 << 0)

// BLE
#define WAIT_FOR_CONNECTION  (1 << 1)
#define WAIT_FOR_DISCONNECT  (1 << 2)
#define WAIT_FOR_SNTP        (1 << 3)
#define WAIT_FOR_WRITE       (1 << 4)

/**********************************************************
*                                                   TYPES *
**********************************************************/
typedef uint32_t sync_event_t;

/**********************************************************
*                                                 GLOBALS *
**********************************************************/
void init_signals_core();

int  poll_event(sync_event_t event, uint32_t timeout);
void signal_event(sync_event_t event);
