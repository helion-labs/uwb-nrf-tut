#include <zephyr.h>
#include <stdio.h>
#include <stdint.h>

#include "global_defines.h"
#include "time_core.h"
#include "master_core.h"
#include "emit_core.h"
#include "signals_core.h"
#include "state_core.h"
#include "flash_core.h"
#include "gpio_core.h"
#include "ble_core.h"
#include "uwb_core.h"

/**********************************************************
*                                                 STATICS *
**********************************************************/
static const char TAG[] = "EMIT_CORE";

K_THREAD_STACK_DEFINE(emit_thread_area, STACK_SIZE_MASTER);
static struct k_thread emit_thread_data;  


/**********************************************************
*                                                FORWARDS *
**********************************************************/
static state_t func_check_status();
static state_t func_scan_advertise();

/**********************************************************
*                                          IMPLEMENTATION *
**********************************************************/
static void next_state_func(state_t* curr_state, state_event_t event) {
    if (!curr_state) {
        ESP_LOGE(TAG, "ARG==NULL!");
        ASSERT(0);
    }
}

// All events are FROM emit core TO emit core
static bool event_filter_func(state_event_t event) {
    return false;
}

static state_array_s func_translation_table[emit_state_len] = {
  { func_check_status   , 30              , NULL },
  { func_scan_advertise , STATE_DONT_LOOP , NULL },
};

static char* event_print_func(state_event_t event) {
    return NULL;
}


static state_init_s* get_emit_state_handle() {

    // variaous statics required for state operation
    static char __aligned(4) emit_msgq_buffer[GENERIC_MESSGE_Q_DEPTH * sizeof(state_t)];
    static struct k_msgq emit_msgq;
    k_msgq_init(&emit_msgq, emit_msgq_buffer, sizeof(state_t), GENERIC_MESSGE_Q_DEPTH);

    static state_init_s state = {
        .next_state                       = next_state_func,
        .translation_table                = func_translation_table,
        .starting_state                   = emit_state_check_status,
        .event_print                      = event_print_func,
        .state_name_string                = "emit_state",
        .filter_event                     = event_filter_func,
        .total_states                     = emit_state_len,
        .state_queue_input_handle         = &emit_msgq,
    };
    return &(state);
}

void init_emit_core(){
  start_new_state_machine(get_emit_state_handle(), &emit_thread_data, emit_thread_area, STACK_SIZE_EMIT);
}

// this relies on the looping feature of the state machine
// to work. (will get called every X seconds)
static state_t func_check_status() {
  ESP_LOGI(TAG, "Entering SNTP State");
  if (get_current_time() == MAX_TIME){
     ESP_LOGI(TAG, "No time, fetching from ble!");
     state_post_event(MASTER_EVENT_SNTP);
     goto pend_sntp_worker;
  }

  if (get_time_age() > STALE_TIME_COUNT){
     ESP_LOGI(TAG, "time stale, fetching from ble!");
     state_post_event(MASTER_EVENT_SNTP);
     goto pend_sntp_worker;
  } 

  return emit_state_scan_advertise;

pend_sntp_worker:
  if(poll_event(WAIT_FOR_WORKER, WAIT_PERIOD_SNTP)){
    ESP_LOGI(TAG, "Timeout waiting for SNTP worker!");
    ASSERT(0); // no recovery here - state machine out of whack
  } else {
    ESP_LOGE(TAG, "SNTP worker done!");
  }
  return NULL_STATE; 

pend_dump_worker:
  if(poll_event(WAIT_FOR_WORKER, WAIT_PERIOD_DUMP)){
    ESP_LOGI(TAG, "Timeout waiting for dump worker!");
    ASSERT(0); // no recovery here - state machine out of whack
  } else {
    ESP_LOGE(TAG, "Dump worker done!");
  }
  return NULL_STATE; 
}


static state_t func_scan_advertise(){
  int time_past  = time_past_previous_advertise_cycle();
  ESP_LOGI(TAG, "%d Seconds passed from previous advertising cycle, current time %u", time_past, get_current_time());

  //Scan UWB 
  uwb_scan_adv();

  // If we have more than 2 valid pages, upload the first valid page 
  get_oldest_page();
  state_post_event(MASTER_EVENT_DUMP); 
   
  if(poll_event(WAIT_FOR_WORKER, WAIT_PERIOD_DUMP)){
    ESP_LOGI(TAG, "Timeout waiting for dump worker!");
    ASSERT(0); // no recovery here - state machine out of whack
  } else {
    ESP_LOGI(TAG, "Dump worker done!");
  }

  k_sleep(K_SECONDS(SLEEP_PERIOD)); 
  return emit_state_check_status;
} 
