#pragma once

#include <stdint.h>

/**********************************************************
*                                                 DEFINES *
**********************************************************/
#define MAX_TIME          (0xFFFFFFFF)
#define SECONDS_IN_HOUR   (3600)
#define DATE_SANITY_CHECK (1635204377) // Time sometime on October 25,2021.
                                       // we can use this as a sniff test since all dates RXed by
                                       // via SNTP _MUST_ be later than this.. In the words of
                                       // the great James Hetfield, "time marches on..!"

#define STALE_TIME_COUNT  (86400)      // After a full day, resync SNTP time to account
                                       // for drift

#define TIME_BETWEEN_ADVERTISE_CYCLES  (60)
#define LATEST_START_ADVERTIING        (7)


// we will try to advertise every time (current_time % TIME_BETWEEN_ADVERTISE_CYCLES) == 0,
// this allows the devices to sync up advertising/sleep periods in a simple way
//
//    x-----------)-----------]--------//---------x-----------)-----------]
// (start)        |           |                   |           |           |
//                |           |                   |           |        (cycle ends)
//                |    (cycle ends)               |    (latest we can start AV for this cycle)
//  (latest we can start advertising in cycle)    |
//                                     (next adverting cycle)


/**********************************************************
*                                                   TYPES *
**********************************************************/

/**********************************************************
*                                                 GLOBALS *
**********************************************************/
uint32_t get_current_time();
int      set_current_time(uint32_t time);
uint32_t get_time_age();
uint32_t time_past_previous_advertise_cycle();
uint32_t time_utill_next_advertise_cycle();
uint32_t get_age_in_hours();
