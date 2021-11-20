#include <zephyr.h>
#include <stdio.h>
#include <stdint.h>

#include "global_defines.h"
#include "master_core.h"
#include "state_core.h"
#include "signals_core.h"
#include "ble_core.h"
#include "upload_shim.h"

/**********************************************************
*                                                 STATICS *
**********************************************************/
static const char TAG[] = "MASTER_CORE";

K_THREAD_STACK_DEFINE(master_thread_area, STACK_SIZE_MASTER);
static struct k_thread master_thread_data;  

/**********************************************************
*                                                FORWARDS *
**********************************************************/
static state_t func_idle();

static state_t func_sntp();
static void    func_sntp_cleanup();

static state_t func_dump();
static void    func_dump_cleanup();

/**********************************************************
*                                          IMPLEMENTATION *
**********************************************************/
static void next_state_func(state_t* curr_state, state_event_t event) {
    if (!curr_state) {
        ESP_LOGE(TAG, "ARG==NULL!");
        ASSERT(0);
    }

    if (*curr_state == master_state_idle) {
        if (event == MASTER_EVENT_SNTP) {
            ESP_LOGI(TAG, "Old State: (Idle) , Next: (SNTP)");
            *curr_state = master_state_sntp;
            return;
        }

        if (event == MASTER_EVENT_DUMP) {
            ESP_LOGI(TAG, "Old State: (Idle) , Next: (DUMP)");
            *curr_state = master_state_dump;
            return;
        }
    }
}

// All events are FROM emit core TO master core
static bool event_filter_func(state_event_t event) {
    return true;
}

static state_array_s func_translation_table[master_state_len] = {
  { func_idle , STATE_DONT_LOOP, NULL              },
  { func_sntp , STATE_DONT_LOOP, func_sntp_cleanup },
  { func_dump , STATE_DONT_LOOP, func_dump_cleanup },
};


static char* event_print_func(state_event_t event) {
    return NULL;
}

static state_init_s* get_master_state_handle() {

    // variaous statics required for state operation
    static char __aligned(4) master_msgq_buffer[GENERIC_MESSGE_Q_DEPTH * sizeof(state_t)];
    static struct k_msgq master_msgq;
    k_msgq_init(&master_msgq, master_msgq_buffer, sizeof(state_t), GENERIC_MESSGE_Q_DEPTH);

    static state_init_s state = {
        .next_state                       = next_state_func,
        .translation_table                = func_translation_table,
        .starting_state                   = master_state_idle,
        .event_print                      = event_print_func,
        .state_name_string                = "master_state",
        .filter_event                     = event_filter_func,
        .total_states                     = master_state_len,
        .state_queue_input_handle         = &master_msgq,
    };
    return &(state);
}

void init_master_core(){
  start_new_state_machine(get_master_state_handle(), &master_thread_data, master_thread_area, STACK_SIZE_MASTER);
}

/**********************************************************
*                                                  STATES *
**********************************************************/

static state_t func_idle() {
  ESP_LOGI(TAG, "Entering Idle State");
  
  return NULL_STATE;
}

static state_t func_sntp() {
  ESP_LOGI(TAG, "Entering SNTP State");
  if (0 == connect_core_device()){
    get_nw_time();
  }
 
  return master_state_idle;
}

static void func_sntp_cleanup() {
  ESP_LOGI(TAG, "Entering SNTP Cleanup");
  connection_cleanup();
  
  signal_event(WAIT_FOR_WORKER); 
}

static state_t func_dump() {
  ESP_LOGI(TAG, "Entering DUMP State");
  if (0 == connect_core_device()){
#ifndef DEBUG_CONFIG_DUMP_TO_BLE
    upload_chunk();
#else 
    // Debug mode, dumps from a big buffer
    // to BLE, bypasses flash
    upload_from_trace();
#endif 
  }
 
  return master_state_idle;
}

static void func_dump_cleanup() {
  ESP_LOGI(TAG, "Entering DUMP Cleanup");
  connection_cleanup();
  
  signal_event(WAIT_FOR_WORKER); 
}
