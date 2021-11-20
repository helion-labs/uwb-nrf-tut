#include <zephyr.h>
#include <stdint.h>

#include "global_defines.h"
#include "signals_core.h"

//TODO: This entire module is stupid - current OS is missing
// k_event structure (not yet ported) so we need to 
// replicate it here, once ported, this module should
// be upated to use k_event

/**********************************************************
*                                                 STATICS *
**********************************************************/
static const char TAG[] = "SYNC_CORE";


//WAIT_FOR_WORKER
static struct k_poll_signal wait_for_worker;
static struct k_poll_event events_wait_for_worker[TOTAL_NUMBER_OF_EVENTS] = {
      K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                      K_POLL_MODE_NOTIFY_ONLY,
                                      &wait_for_worker,
                                      0)
  };

//WAIT_FOR_CONNECTION
static struct k_poll_signal wait_for_connection;
static struct k_poll_event events_wait_for_connection[TOTAL_NUMBER_OF_EVENTS] = {
      K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                      K_POLL_MODE_NOTIFY_ONLY,
                                      &wait_for_connection,
                                      0)
  };


//WAIT_FOR_SNTP
static struct k_poll_signal wait_for_sntp;
static struct k_poll_event events_wait_for_sntp[TOTAL_NUMBER_OF_EVENTS] = {
      K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                      K_POLL_MODE_NOTIFY_ONLY,
                                      &wait_for_sntp,
                                      0)
  };

//WAIT_FOR_DISCONNECT
static struct k_poll_signal wait_for_disconnect;
static struct k_poll_event events_wait_for_disconnect[TOTAL_NUMBER_OF_EVENTS] = {
      K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                      K_POLL_MODE_NOTIFY_ONLY,
                                      &wait_for_disconnect,
                                      0)
  };

//WAIT_FOR_WRITE
static struct k_poll_signal wait_for_write;
static struct k_poll_event events_wait_for_write[TOTAL_NUMBER_OF_EVENTS] = {
      K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                      K_POLL_MODE_NOTIFY_ONLY,
                                      &wait_for_write,
                                      0)
  };




/**********************************************************
*                                                FORWARDS *
**********************************************************/

/**********************************************************
*                                          IMPLEMENTATION *
**********************************************************/

// Wait for worker
void init_signals_core(){
  k_poll_signal_init(&wait_for_worker);
  k_poll_signal_init(&wait_for_connection);
  k_poll_signal_init(&wait_for_sntp);
  k_poll_signal_init(&wait_for_disconnect);
  k_poll_signal_init(&wait_for_write);
}

static int poll_worker_done(uint32_t timeout){
  int ret = 0;
  k_poll_signal_reset(&wait_for_worker);
  events_wait_for_worker[0].state = K_POLL_STATE_NOT_READY;
  if(0 != k_poll(events_wait_for_worker, TOTAL_NUMBER_OF_EVENTS, K_SECONDS(timeout))){
    ret = 1; 
  }
  k_poll_signal_reset(&wait_for_worker);
  events_wait_for_worker[0].state = K_POLL_STATE_NOT_READY;
  return ret;
}
static void signal_worker_done(){
  k_poll_signal_raise(&wait_for_worker, DONE_VAL);
}

// Wait for BLE connection
static int poll_connection(uint32_t timeout){
  int ret = 0;
  k_poll_signal_reset(&wait_for_connection);
  events_wait_for_connection[0].state = K_POLL_STATE_NOT_READY;
  if(0 != k_poll(events_wait_for_connection, TOTAL_NUMBER_OF_EVENTS, K_SECONDS(timeout))){
    ret = 1; 
  }
  k_poll_signal_reset(&wait_for_connection);
  events_wait_for_connection[0].state = K_POLL_STATE_NOT_READY;
  return ret;
}
static void signal_connection_done(){
  k_poll_signal_raise(&wait_for_connection, DONE_VAL);
}

// Wait for SNTP (read from ESP GATT server)
static int poll_sntp(uint32_t timeout){
  int ret = 0;
  k_poll_signal_reset(&wait_for_sntp);
  events_wait_for_sntp[0].state = K_POLL_STATE_NOT_READY;
  if(0 != k_poll(events_wait_for_sntp, TOTAL_NUMBER_OF_EVENTS, K_SECONDS(timeout))){
    ret = 1; 
  }
  k_poll_signal_reset(&wait_for_sntp);
  events_wait_for_sntp[0].state = K_POLL_STATE_NOT_READY;
  return ret;
}
static void signal_sntp_done(){
  k_poll_signal_raise(&wait_for_sntp, DONE_VAL);
}

// Wait for disconnect from remote
static int poll_disconnect(uint32_t timeout){
  int ret = 0;
  k_poll_signal_reset(&wait_for_disconnect);
  events_wait_for_disconnect[0].state = K_POLL_STATE_NOT_READY;
  if(0 != k_poll(events_wait_for_disconnect, TOTAL_NUMBER_OF_EVENTS, K_SECONDS(timeout))){
    ret = 1; 
  }
  k_poll_signal_reset(&wait_for_disconnect);
  events_wait_for_disconnect[0].state = K_POLL_STATE_NOT_READY;
  return ret;
}
static void signal_disconnect_done(){
  k_poll_signal_raise(&wait_for_disconnect, DONE_VAL);
}

// Wait for write
static int poll_write(uint32_t timeout){
  int ret = 0;
  k_poll_signal_reset(&wait_for_write);
  events_wait_for_write[0].state = K_POLL_STATE_NOT_READY;
  if(0 != k_poll(events_wait_for_write, TOTAL_NUMBER_OF_EVENTS, K_SECONDS(timeout))){
    ret = 1; 
  }
  k_poll_signal_reset(&wait_for_write);
  events_wait_for_write[0].state = K_POLL_STATE_NOT_READY;
  return ret;
}
static void signal_write_done(){
  k_poll_signal_raise(&wait_for_write, DONE_VAL);
}

int poll_event(sync_event_t event, uint32_t timeout){
  switch(event){
    case WAIT_FOR_WORKER:
      return poll_worker_done(timeout);
      break;
    case WAIT_FOR_CONNECTION:
      return poll_connection(timeout);
      break;
    case WAIT_FOR_SNTP:
      return poll_sntp(timeout);
      break;
    case WAIT_FOR_DISCONNECT:
      return poll_disconnect(timeout);
      break;
    case WAIT_FOR_WRITE:
      return poll_write(timeout);
      break;
    default:
      ESP_LOGE(TAG, "unknown sync event requested");
      ASSERT(0);
  };

  // will never get here
  return 0; // stop the compiler from complaining 
}

void signal_event(sync_event_t event){
   switch(event){
    case WAIT_FOR_WORKER:
      signal_worker_done();
      break;
    case WAIT_FOR_CONNECTION:
      signal_connection_done();
      break;
    case WAIT_FOR_SNTP:
      signal_sntp_done();
      break;
    case WAIT_FOR_DISCONNECT:
      signal_disconnect_done();
      break;
    case WAIT_FOR_WRITE:
      signal_write_done();
      break;
    default:
      ESP_LOGE(TAG, "unknown sync event requested");
      ASSERT(0);
  }; 
}
