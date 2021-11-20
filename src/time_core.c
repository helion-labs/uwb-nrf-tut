#include <zephyr.h>
#include <drivers/flash.h>
#include <storage/flash_map.h>
#include <device.h>
#include <stdio.h>
#include <stdint.h>

#include "global_defines.h"
#include "time_core.h"

/**********************************************************
*                                                 STATICS *
**********************************************************/
static const char TAG[] = "TIME_CORE";
static uint32_t previous_unix_time;
static uint32_t uptime_at_previous_unix_time;

K_MUTEX_DEFINE(time_sem);

/**********************************************************
*                                          IMPLEMENTATION *
**********************************************************/

// this sets the system time, since we don't have an RTC
// we get the time, in seconds from epoch (unix time) 
// from the central ESP32
// Returns -> 0, time good
// Returns -> 1, something fishy with time
int set_current_time(uint32_t time){
  if (time == 0 || time == MAX_TIME){
    ESP_LOGE(TAG, "Time makes no sense! (zero or max?) %u", time);
    return 1;
  }

  if (DATE_SANITY_CHECK > time){
    ESP_LOGE(TAG, "Time makes no sense! %u (too small?)", time);
    return 1;
  } 

  k_mutex_lock(&time_sem, K_FOREVER);
  previous_unix_time = time;
  uptime_at_previous_unix_time = k_uptime_get()/1000;
  ESP_LOGI(TAG, "Setting time to %u, at uptime, %u ", previous_unix_time, uptime_at_previous_unix_time);
  k_mutex_unlock(&time_sem);
  
  return 0;
}

uint32_t get_current_time(){
  uint32_t ret;
  k_mutex_lock(&time_sem, K_FOREVER);
  if ( previous_unix_time == 0 ) {
    ESP_LOGI(TAG, "Tried to get time before we init! \n");
    ret = MAX_TIME;
  } else {
    ret = previous_unix_time + k_uptime_get()/1000 - uptime_at_previous_unix_time; 
  }

  k_mutex_unlock(&time_sem);
  return ret;
}

// How long ago we got the current time
uint32_t get_time_age(){
  uint32_t ret;
  k_mutex_lock(&time_sem, K_FOREVER);
  if ( previous_unix_time == 0 ) {
    ESP_LOGI(TAG, "Tried to get time before we init! \n");
    ret = MAX_TIME;
  } else {
    ret = k_uptime_get()/1000 - uptime_at_previous_unix_time; 
  }

  k_mutex_unlock(&time_sem);
  return ret;
}

uint32_t time_past_previous_advertise_cycle(){
  k_mutex_lock(&time_sem, K_FOREVER);
  uint32_t ret = get_current_time() % TIME_BETWEEN_ADVERTISE_CYCLES; 
  k_mutex_unlock(&time_sem);
  return ret;
}

uint32_t time_utill_next_advertise_cycle(){
  k_mutex_lock(&time_sem, K_FOREVER);
  uint32_t ret = TIME_BETWEEN_ADVERTISE_CYCLES - time_past_previous_advertise_cycle();
  k_mutex_unlock(&time_sem);
  return ret;
}

uint32_t get_age_in_hours(uint32_t time){
  int age_seconds = get_current_time() - time;
  if (0 > age_seconds){
    ESP_LOGE(TAG, "Age is negative!?!");
    ASSERT(0);
  }

  return age_seconds/SECONDS_IN_HOUR; 
}
