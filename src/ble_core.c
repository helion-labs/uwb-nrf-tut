#include <zephyr.h>
#include <drivers/flash.h>
#include <storage/flash_map.h>
#include <device.h>
#include <stdio.h>
#include <stdint.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>
#include <bluetooth/scan.h>

#include "global_defines.h"
#include "ble_core.h"
#include "trace_core.h"
#include "signals_core.h"
#include "time_core.h"

/**********************************************************
*                                                 STATICS *
**********************************************************/
static const char TAG[] = "BLE_CORE";
static handle_key_t uuid_key;
static struct bt_uuid_16  uuid;
static struct bt_gatt_discover_params discover_params;
static char usr_data[MAX_USER_DATA_LEN];
static struct bt_conn *esp_conn;
static int write_status;
K_MUTEX_DEFINE(sync_sem);

//TODO: properly handle this
static char VAL[] = {0,1,2,3,4,5,6,7,8};
static const struct bt_data advertising_data[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, VAL, 8),
};

/**********************************************************
*                                          IMPLEMENTATION *
**********************************************************/

// if return true,continue parsing
static bool data_cb(struct bt_data *data, void *user_data) {
  adv_packet_t *packet = user_data;

  switch (data->type) {
    case BT_DATA_MANUFACTURER_DATA:
      memcpy(packet->manufactuers_data, data->data, MIN(data->data_len, BLE_MANUFACTURERS_DATA_LEN - 1));
      return true;
    case BT_DATA_NAME_COMPLETE:
      memcpy(packet->name_complete, data->data, MIN(data->data_len, BLE_MANUFACTURERS_NAME_LEN - 1));
      return true;
    default:
      return true;
    }
}

// this tries to sepcifically connect to the ESP32
static void scan_cb_connect(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *buf) {
  adv_packet_t packet;
  memset(usr_data, 0 , MAX_USER_DATA_LEN);
  memset(&packet, 0, sizeof(packet));

  bt_data_parse(buf, data_cb, &packet);
  
  /* We're only interested in connectable events */
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}


  // Only try to connect to the ESP32 core device 
  if (0 == strncmp(packet.name_complete, CORE_DEVICE_NAME, CORE_DEVICE_LEN)){
    ESP_LOGI(TAG, "Name of device: %s", packet.name_complete);
    ESP_LOGI(TAG, "Found a core device!");
  } else {
    return;
  }

	if (bt_le_scan_stop()) {
		return;
	}

	int err;
  struct bt_conn *conn;
	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				BT_LE_CONN_PARAM_DEFAULT, &conn);

  //todo - handle err (rekick connection?)
}


// this only records advertising packets
static void scan_cb_record(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *buf) {
  adv_packet_t packet;
  memset(usr_data, 0 , MAX_USER_DATA_LEN);
  memset(&packet, 0, sizeof(packet));

  packet.RSSI = rssi;
  bt_data_parse(buf, data_cb, &packet);

  // Push the advertsting packet to trace core
  submit_adv_packet(&packet);
}

static int start_adv(){
	int err = bt_le_adv_start(BT_LE_ADV_NCONN, advertising_data, ARRAY_SIZE(advertising_data),
			      NULL, 0);
	if (err) {
		ESP_LOGE(TAG, "Advertising failed to start (err %d)\n", err);
	}
  return err;
}

static void stop_adv(){
  int err = bt_le_adv_stop();
	if (err) {
		ESP_LOGE(TAG, "Advertising failed to stop (err %d)\n", err);
	}
}

static void start_scan(void){
	int err;
	ESP_LOGI(TAG, "Starting scanning (no connection)!");

  // don't ask for scan response, don't try to connect
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, scan_cb_record);
	if (err) {
		ESP_LOGE(TAG,"Scanning failed to start (err %d)\n", err);
		return;
	}

	ESP_LOGI(TAG, "Scanning successfully started\n");
}

static int generate_fake_incidence(){
  ESP_LOGI(TAG, "Generating fake scanning data!");
  for(int i = 0; i < 300; i++){
    adv_packet_t packet;
    memset(usr_data, 0 , MAX_USER_DATA_LEN);
    memset(&packet, 0, sizeof(packet));
    memcpy(packet.name_complete, DEVICE_NAME, DEVICE_NAME_LEN);

    // fake the advertising data
    k_sleep(K_MSEC(3));
    packet.manufactuers_data[3] = k_uptime_get();
    k_sleep(K_MSEC(3));
    packet.manufactuers_data[1] = k_uptime_get();

    // Push the advertsting packet to trace core
    submit_adv_packet(&packet);
  }
}

static int start_scan_connect(void){
	int err;
	ESP_LOGI(TAG, "Starting scanning + connecting \n");

  // don't ask for scan response
	struct bt_le_scan_param scan_param = {
		.type       = BT_HCI_LE_SCAN_PASSIVE,
		.options    = BT_LE_SCAN_OPT_NONE,
		.interval   = 0x0010,
		.window     = 0x0010,
	};

	err = bt_le_scan_start(&scan_param, scan_cb_connect);
	if (err) {
		ESP_LOGE(TAG, "Scanning failed to start (err %d)\n", err);
	}

	ESP_LOGI(TAG, "Scanning successfully started\n");
	return err;
}

static void stop_scan(void){
	int err;
	ESP_LOGI(TAG, "stopping scanning! \n");

	err = bt_le_scan_stop();
  if (!err){
    return;
  }

	ESP_LOGI(TAG, "failled to stop scanning: error %d \n", err);
}

static void disconnect_remote(){
  if(esp_conn){
    bt_conn_disconnect(esp_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
  }
}

// scans for advertising packets.
// does not attempt to connect
int start_scan_record(){
#ifndef BLE_TEST_MODE
  set_blue_led(1);
  start_scan();
  start_adv();
	k_sleep(K_SECONDS(SCAN_DURATION_SECONDS));
  stop_scan();
  stop_adv();
  set_blue_led(0);
#else
  generate_fake_incidence();
#endif

  return 0;
}

static int start_scan_and_connect(){
  return start_scan_connect();
}

static void init_kernel_objects(){

}

static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params) {
	int err;

	if (!attr) {
		return BT_GATT_ITER_STOP;
	}

  // print out the UUID details
  memset(usr_data, 0, MAX_USER_DATA_LEN);
  bt_uuid_to_str(attr->uuid, usr_data, MAX_USER_DATA_LEN);
	ESP_LOGI(TAG, "[ATTRIBUTE] handle %u", attr->handle);
	ESP_LOGI(TAG,"[ATTRIBUTE] uuid %s", usr_data);

 
 // see: https://github.com/zephyrproject-rtos/zephyr/issues/35363
 // The reason why are you getting the "wrong" handle is
 // because a characteristic is split into two attributes,
 // the characteristic declaration, and the characteristic value.
 // When doing the Discover Characteristics by UUID procedure
 // you will receive the characteristic declaration attribute, 
 // the value of this attribute contains
 // the handle of the characteristic value attribute.

	if (bt_uuid_cmp(discover_params.uuid, BT_UUID_SERVICE_CW)==0) {
    ESP_LOGI(TAG, "Discovered CW-100 service!");  
    memcpy(&uuid, BT_UUID_TIME_VALUE, sizeof(uuid));
    discover_params.uuid = &uuid.uuid;
  	discover_params.start_handle = attr->handle + 1;
	  discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

	  err = bt_gatt_discover(conn, &discover_params);
	  if (err) {
		  ESP_LOGE(TAG, "Discover failed (err %d)\n", err);
	  }
  } else if (bt_uuid_cmp(discover_params.uuid, BT_UUID_TIME_VALUE)==0) { 
	  uuid_key.time = attr->handle + 1;
    ESP_LOGI(TAG, "Discovered TIME characteristic");  
		
    memcpy(&uuid, BT_UUID_DUMP_VALUE, sizeof(uuid));
    discover_params.uuid = &uuid.uuid;
    discover_params.start_handle = attr->handle + 1;
    discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
    err = bt_gatt_discover(conn, &discover_params);
	  if (err) {
		  ESP_LOGI(TAG, "Discover failed (err %d)\n", err);
	  }
  } else if (bt_uuid_cmp(discover_params.uuid, BT_UUID_DUMP_VALUE)==0) { 
    ESP_LOGI(TAG, "Discovered DUMP characteristic");  
	  uuid_key.dump = attr->handle + 1;

		memcpy(&uuid, BT_UUID_FW_VER_VALUE, sizeof(uuid));
    discover_params.uuid = &uuid.uuid;
    discover_params.start_handle = attr->handle + 1;
    discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
    err = bt_gatt_discover(conn, &discover_params);
	  if (err) {
		  ESP_LOGE(TAG, "Discover failed (err %d)\n", err);
	  }
    // At this point, the connection should be ready for use
    signal_event(WAIT_FOR_CONNECTION);
    return BT_GATT_ITER_STOP;
  }

  return BT_GATT_ITER_STOP;
}

static void connected(struct bt_conn *conn, uint8_t err) {
  memset(usr_data, 0, sizeof(usr_data)); 
 
  if (err) {
    ESP_LOGE(TAG, "Connection failed (err %u) \n", err);
  } else {
    ESP_LOGI(TAG, "Connected \n");
    esp_conn = conn;
  }
	
  bt_addr_le_to_str(bt_conn_get_dst(conn), usr_data, sizeof(usr_data));
  ESP_LOGI(TAG, "Connected: %s\n", usr_data);
 
  memset(&discover_params, 0, sizeof(discover_params));
  if (conn) {
    memcpy(&uuid, BT_UUID_SERVICE_CW, sizeof(uuid));
    discover_params.uuid = &uuid.uuid;
    discover_params.func = discover_func; 
    discover_params.start_handle = BT_ATT_FIRST_ATTTRIBUTE_HANDLE; 
    discover_params.end_handle = BT_ATT_LAST_ATTTRIBUTE_HANDLE; 
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    err = bt_gatt_discover(conn, &discover_params); 
    if (err) { 
      ESP_LOGE(TAG, "Discover failed(err %d)\n", err); 
      return; 
    }
  }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != esp_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));


	bt_conn_unref(esp_conn);
	esp_conn = NULL;
  
	ESP_LOGI(TAG, "Disconnected: %s (reason 0x%02x) - signalling disconection", addr, reason);
#if 0
  signal_event(WAIT_FOR_DISCONNECT);
#endif 
}

static struct bt_conn_cb conn_callbacks = { 
  .connected = connected,
  .disconnected = disconnected,
};

void init_ble_core(){
  int err;

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		ESP_LOGE(TAG, "Bluetooth init failed (err %d)\n", err);
	  ASSERT(0);
  }

  // register connect/disconnect callback
  bt_conn_cb_register(&conn_callbacks);

  init_kernel_objects();
}

static void callback_write_dump(struct bt_conn *conn, uint8_t err,
				     struct bt_gatt_write_params *params){

	ESP_LOGI(TAG, "[write] error  %d", err);
	ESP_LOGI(TAG, "[write] handle %d", params->handle);
  
  // set global (does this need a memory barrier?)
  write_status = 1;

  if(err){
    ESP_LOGI(TAG, "Error on write!");
    signal_event(WAIT_FOR_WRITE);
  }

  //TODO: do we need a mutex for the key?
  if (params->handle != uuid_key.dump){
    ESP_LOGI(TAG, "Unknown write?!");
  } else {
    write_status = 0;
    signal_event(WAIT_FOR_WRITE);
  }
}

static uint8_t callback_read_network(struct bt_conn *conn, uint8_t err,
				    struct bt_gatt_read_params *params,
				    const void *data, uint16_t length){
 
	ESP_LOGI(TAG, "[READ] error  %d", err);
	ESP_LOGI(TAG, "[READ] lenght %d", length);
 
  if (!err){
    if (params->handle_count == SINGLE_READ){
      if (params->single.handle == uuid_key.time){  
          uint32_t time = *(uint32_t*)(data);
	        ESP_LOGI(TAG, "[READ] time %u\n", time);
          
          // Time core can chose to ignore the time (ie, if it's zero, etc)
          if (set_current_time(time) == 0){
            signal_event(WAIT_FOR_SNTP);
          } else {
            ESP_LOGE(TAG, "Something fishy new STNP time...!");
          }
          // don't keep on reading
          return BT_GATT_ITER_STOP; 
       }
    }
  }
  return BT_GATT_ITER_CONTINUE;
}


// requires connection
int get_nw_time(){
  // Enqueue the read
  struct bt_gatt_read_params read_parms;
  memset(&read_parms, 0, sizeof(read_parms));
  read_parms.func          = callback_read_network;
  read_parms.handle_count  = SINGLE_READ ;
  read_parms.single.handle = uuid_key.time;
  read_parms.single.offset = 0; //offset into read
  bt_gatt_read(esp_conn, &read_parms); 

  // Wait for read to come through  
  if(poll_event(WAIT_FOR_SNTP, CONNECTION_TIMEOUT)){
    // Failed to get SNTP time
    ESP_LOGE(TAG, "Failed to get SNTP time");
    return 0;
  } else {
    ESP_LOGI(TAG, "Set SNTP time");
  }

  return 1;
}

// returns 0 on good, 1 on error
int connect_core_device(){
  if (start_scan_and_connect()){
    ESP_LOGI(TAG, "Failed to start scanning!");
    return 1; 
  }

  // Wait for Connection + GATT discovery  
  if(poll_event(WAIT_FOR_CONNECTION, CONNECTION_TIMEOUT)){
    ESP_LOGI(TAG, "Failed to connect with ESP!");
    signal_event(WAIT_FOR_SNTP);
    return 1;
  }

  if(!esp_conn){
    ESP_LOGI(TAG, "Can't connect to ESP32 - esp_conn null!");
    signal_event(WAIT_FOR_SNTP);
    return 1;
  }

  return 0;
}

void connection_cleanup(){
  // make sure we disconnect and also stop scanning
  disconnect_remote();
  
  // normal for this to bug out if we already stopped scanning
  stop_scan();
 
// for POC, we will assume disconnect always works
// as the logic for handling remote disconnect
// is not trival

#if 0 
  // Wait for disconnect callback
  if(poll_event(WAIT_FOR_DISCONNECT, DISCONNECT_TIMEOUT)){
    // Failed to get SNTP time
    ESP_LOGE(TAG, "Failed to disconnect - no recovery");
    ASSERT(0);
  } else {
    ESP_LOGI(TAG, "Disconnection done!");
  }
#endif 
}

// upload a BLE incident chunk 
// to the core device
int ble_dump_chunk(uint8_t * chunk, size_t len){
  struct bt_gatt_write_params write_parms;
  memset(&write_parms, 0, sizeof(write_parms));
  write_status = 1; // will be set in callback

  // enqueue write
  ESP_LOGI(TAG, "Writting data of len %d to handle %d", len, uuid_key.dump);
  write_parms.func   = callback_write_dump;
  write_parms.offset = 0;
  write_parms.length = len;
  write_parms.data   = chunk;
  write_parms.handle = uuid_key.dump; 
  bt_gatt_write(esp_conn, &write_parms);

  // Wait for write to go through
  if(poll_event(WAIT_FOR_WRITE, WRITE_TIMEOUT)){
    // Failed to dump data
    ESP_LOGE(TAG, "Failed to write!");
  } else {
    ESP_LOGI(TAG, "Write done!");
  }

  // set in callbacks
  return write_status;
}
