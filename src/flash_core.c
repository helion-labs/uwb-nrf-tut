#include <zephyr.h>
#include <drivers/flash.h>
#include <storage/flash_map.h>
#include <device.h>
#include <stdio.h>
#include <string.h>

#include "ble_core.h"
#include "global_defines.h"
#include "flash_core.h"
#include "time_core.h"

/**********************************************************
*                                                 STATICS *
**********************************************************/
static const char     TAG[] = "FLASH_CORE";
static uint8_t flash_memblock[FLASH_PAGE_SIZE];

static flash_curr_t   curr;
static flash_oldest_t oldest_info;

static bool flash_has_space;
static void set_flash_full(bool full);

static struct k_mutex flash_mutex;
static const struct device *flash_dev;

static uint8_t uploaded[UPLOAD_CHUNKS_IN_PAGE];

/**********************************************************
*                                                FORWARDS *
**********************************************************/
static int sync_flash_status();
static int set_next_packet_address();

/**********************************************************
*                                          IMPLEMENTATION *
**********************************************************/
static void init_kernel_objects(){
  k_mutex_init(&flash_mutex);
}

void init_flash_core(){
  ESP_LOGI(TAG, "Initing flash core!");
  flash_dev = device_get_binding(DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL);

  if (!flash_dev) {
    ESP_LOGE(TAG, "Nordic nRF5 flash driver was not found!");
    ASSERT(0);  
  } 
 
  init_kernel_objects();

  // for the tutorial, don't get the flash contex
#if 0
  // find the first place in flash we can write to
  if(sync_flash_status()){
    set_flash_full(true);
  }else{
    set_flash_full(false);
  }
 
  // set up the structures that keep track of the oldest page (first upload candidate)
  get_oldest_page();
#endif 

#ifdef TEST_MODE_FLASH
  flash_test();
#endif 
}

void print_hex(uint8_t* buffer, size_t buff_len) {
#ifdef CONFIG_LOG
    if (buff_len == 0) {
        return;
    }
    #define BYTES_PER_LINE 16
    char temp_buffer[BYTES_PER_LINE + 3];
    char hex_buffer[3 * BYTES_PER_LINE + 1];
    const char *ptr_line;
    int bytes_cur_line;

    do {
        if (buff_len > BYTES_PER_LINE) {
            bytes_cur_line = BYTES_PER_LINE;
        } else {
            bytes_cur_line = buff_len;
        }
        if (true) {
            //use memcpy to get around alignment issue
            memcpy(temp_buffer, buffer, (bytes_cur_line + 3) / 4 * 4);
            ptr_line = temp_buffer;
        } else {
            ptr_line = buffer;
        }

        for (int i = 0; i < bytes_cur_line; i ++) {
            sprintf(hex_buffer + 3 * i, "%02x ", ptr_line[i]);
        }
        ESP_LOGI(TAG, "%s", hex_buffer);
        buffer += bytes_cur_line;
        buff_len -= bytes_cur_line;
    } while (buff_len);
#endif 
}

void take_flash_sem(){
  k_mutex_lock(&flash_mutex, K_FOREVER);
}

void give_flash_sem(){
  k_mutex_unlock(&flash_mutex);
}

bool get_flash_full(){
  bool ret;
  ret = flash_has_space;
  return ret;
}

static void set_flash_full(bool full){
  flash_has_space = full;
}

static int increment_to_next_valid_page(){
  flash_packet_t test;
  if(curr.current_valid_page == FINAL_VALID_PAGE){
    curr.current_valid_page = FIRST_PAGE_INCIDENCE_DATA; 
  } else{
    curr.current_valid_page = curr.current_valid_page + 1; 
  }

  //ESP_LOGI(TAG, "next candidate == %d \n", curr.current_valid_page);
  int offset = FLASH_PAGE_SIZE*curr.current_valid_page;
  if (flash_read(flash_dev, offset, &test, sizeof(flash_packet_t)) != 0) {
    ESP_LOGE(TAG, "Failed to read flash \n");
    ASSERT(0); 
  }

  if (test.type == PAGE_HEADER_MAGIC){
      return 1;
  }
  
  // next page not full
  return 0;
}


static void parse_header_packet(flash_packet_t *header, int page){
  if(!header){
    ESP_LOGE(TAG, "ARG NULL!");
    ASSERT(0);
  }
  
  if (page < FIRST_PAGE_INCIDENCE_DATA){
    ESP_LOGE(TAG, "Tried reading page %d - this is not allowed!", page);
    ASSERT(0);
  }

  if(page > MAX_VALID_PAGE){
    ESP_LOGE(TAG, "Tried reading page %d - this is not allowed!", page);
    ASSERT(0); 
  }
  
  int offset = FLASH_PAGE_SIZE*page;
  ESP_LOGI(TAG, "Parsing header for page %d, offset %d", page, offset);
  if (flash_read(flash_dev, offset, header, sizeof(flash_packet_t)) != 0) {
      ESP_LOGE(TAG, "Flash read failed!");
      ASSERT(0);
  }
}

static void drop_header_packet(){
  ESP_LOGI(TAG, "dropping header packet");
  flash_packet_t header_packet;
  memset(&header_packet, 0, sizeof(header_packet));

  header_packet.specifics.id   = curr.current_id + 1; // Smallest valid current ID is 1 
  header_packet.type           = PAGE_HEADER_MAGIC; 
  header_packet.utc            = get_current_time(); 

  // TODO: better handle!
  int offset = FLASH_PAGE_SIZE*curr.current_valid_page;
  if (flash_write(flash_dev, offset, &header_packet, sizeof(flash_packet_t)) != 0) {
      ESP_LOGE(TAG, "Flash write failed!");
  }
}

void write_packet(trace_packet_t* trace_packet){
  flash_packet_t flash_packet;
  memset(&flash_packet, 0, sizeof(flash_packet));

  if (get_flash_full()){
    return;
  }

  int offset = FLASH_PAGE_SIZE*curr.current_valid_page + curr.current_valid_packet_in_page*FLASH_SIZE_PACKET;
  if(curr.current_valid_packet_in_page == HEADER_PACKET_OFFSET){
    ESP_LOGI(TAG, "Writting a (header) packet to %d, page %d packet offset %d, id = %d", 
        offset, curr.current_valid_page, curr.current_valid_packet_in_page, curr.current_id);  

    drop_header_packet();
    curr.total_valid_pages++;
    curr.current_valid_packet_in_page++;
    // recalculate offset for next write
    offset = FLASH_PAGE_SIZE*curr.current_valid_page + curr.current_valid_packet_in_page*FLASH_SIZE_PACKET; 
  }
 
  flash_packet.type   = PAGE_NORMAL_ENTRY_MAGIC; 
  flash_packet.RSSI   = trace_packet->RSSI;
  flash_packet.counts = trace_packet->counts; 
  flash_packet.utc    = trace_packet->utc;
  memcpy(flash_packet.manufactuers_data, trace_packet->manufactuers_data, BLE_MANUFACTURERS_DATA_LEN);
  
  // TODO: better handle!
  ESP_LOGI(TAG, "Writting a packet to %d, page %d packet offset %d", offset, curr.current_valid_page, curr.current_valid_packet_in_page);  
  if (flash_write(flash_dev, offset, &flash_packet, sizeof(flash_packet_t)) != 0) {
      printf("Flash write failed!\n");
  }

  // get the next empty address for the next write
  if(set_next_packet_address()){
    printf("Flash is now full! \n");
    set_flash_full(true);
  }
}

// There are 128 pages in the device,
// out of this, TOTAL_PAGES_FOR_INCIDENCE_DATA, is used for the image,
// there are two banks of possible valid images, bank0 and bank1,
// when FOTA is not happening, the second bank is used to store
// covid tracing information.
//
// The first ever covid tracing information is written to the first page 
// in the second bank, (page == FIRST_PAGE_IMAGE_1), then the next page, 
// and so forth. When uploading, the oldest page is uploaded first. To keep track 
// of the age of the page, there will be a page header in each page, this will
// have a "page_id" field which will be incremented by 1 every time a new page is written
// this value will start at one and go up to uint32_max (although flash will wear out before then)   
      
// return 0, we have space
// return 1, we are full
static int sync_flash_status(){
  ESP_LOGI(TAG, "First page of incidence data = %d offset = %d, total pages = %d",
       FIRST_PAGE_INCIDENCE_DATA, OFFSET_TO_FIRST_INCIDENCE_DATA, TOTAL_PAGES_FOR_INCIDENCE_DATA );
  
  ESP_LOGI(TAG, "Finding first emtpy page \n");

  if (flash_dev == NULL){
    printf("FAILED - flash dev not init! \n");
    ASSERT(0);
  }

  uint32_t offset;
  flash_packet_t test;
  curr.current_valid_page = FIRST_PAGE_INCIDENCE_DATA;

  for(int i = 0; i < TOTAL_PAGES_FOR_INCIDENCE_DATA; i++){
    offset = (FIRST_PAGE_INCIDENCE_DATA + i) * FLASH_PAGE_SIZE;
    printf("Checking page offset = %d, number = %d \n", offset, i + FIRST_PAGE_INCIDENCE_DATA);
    k_sleep(K_MSEC(1)); // the prints above cause issues with RTT - not required
    
    if (flash_read(flash_dev, offset, &test, sizeof(test)) != 0) {
      printf("Failed to read flash \n");
      ASSERT(0); 
    }

    if (test.type == PAGE_HEADER_MAGIC){
      printf("Found a header packet page ID == %d \n", test.specifics.id);
      curr.total_valid_pages++;
      if (test.specifics.id > curr.current_id){
        printf("Found a new candidate for largest id = %d\n" , test.specifics.id);
        curr.current_valid_page = i + FIRST_PAGE_INCIDENCE_DATA;
        curr.current_id = test.specifics.id;
      }
    }
  }

  offset = FLASH_PAGE_SIZE*curr.current_valid_page;
  ESP_LOGI(TAG, "current valid page == %d, find first valid offset in page \n", curr.current_valid_page);
  if (flash_read(flash_dev, offset, flash_memblock, FLASH_PAGE_SIZE) != 0) {
      printf("Failed to read flash");
      ASSERT(0);
  }

  flash_packet_t * packet_ptr = (flash_packet_t*)flash_memblock;
  for(int i = 0; i < PACKETS_IN_PAGE; i++){
     if(packet_ptr->type == PAGE_HEADER_MAGIC_EMTPY){
        ESP_LOGI(TAG, "Found empty packet (packet offset in page) %d in page %d \n", i, curr.current_valid_page);
        curr.current_valid_packet_in_page = i;
        return 0;    
     }
     // increment to next packet
     packet_ptr++;
  }
  
  // this is rare, if we get here, there is a full page, but we didn't get another advertising 
  // packet that would have created the next page.
  if (increment_to_next_valid_page()){
      ESP_LOGE(TAG, "device is full! \n");
      set_flash_full(true);
      return 1;
  }

  curr.current_valid_packet_in_page = 0;
  curr.current_id++;
  ESP_LOGI(TAG, "next empty page, offset = %d, %d \n", curr.current_valid_page, curr.current_valid_packet_in_page);
  return 0;
}

// if true, there is another availible packet in flash
// false - we are full
static int set_next_packet_address(){
  // test if we still have room in the current page
  if (curr.current_valid_packet_in_page != FINAL_VALID_PACKET_OFFSET){
    curr.current_valid_packet_in_page = curr.current_valid_packet_in_page + 1;  
    return 0;
  }

  printf("ran out of packets in page, will try to goto next page! \n");
  if (increment_to_next_valid_page()){
      printf("device is full!");
      set_flash_full(true);
      return 1;
  }

  // next page is valid
  curr.current_valid_packet_in_page = 0;
  curr.current_id++; 
  return 0;
}

//TODO: can we optimize this?
uint32_t get_oldest_page(){
  ESP_LOGI(TAG, "Calculating oldest page!");
  oldest_info.oldest_id = MAX_VALID_ID;
  flash_packet_t test;

  for(int i = 0; i < TOTAL_PAGES_FOR_INCIDENCE_DATA; i++){
    int offset =  OFFSET_TO_FIRST_INCIDENCE_DATA + FLASH_PAGE_SIZE*i;
    ESP_LOGI(TAG, "Checking page == %d (for oldest)", offset);
    
    if (flash_read(flash_dev, offset, &test, sizeof(flash_packet_t)) != 0) {
      ESP_LOGI(TAG, "Failed to read flash");
      ASSERT(0); 
    }

    if (test.type == PAGE_HEADER_MAGIC){
      ESP_LOGI(TAG, "Found a header packet page ID == %d", test.specifics.id);
      if ( test.specifics.id < oldest_info.oldest_id ){
        ESP_LOGI(TAG, "Found a new candidate for oldest page = %d" , test.specifics.id);
        oldest_info.oldest_id = test.specifics.id;
        oldest_info.oldest_id_page = i + FIRST_PAGE_INCIDENCE_DATA;
      }
    }
  }

  ESP_LOGI(TAG, "Oldest page == %u, oldest_id_page == %u", oldest_info.oldest_id, oldest_info.oldest_id_page);
  return oldest_info.oldest_id;
}

// erase a PAGE (not offset!)
static void erase_page(uint32_t id){
  if (id < FIRST_PAGE_INCIDENCE_DATA){
    ESP_LOGE(TAG, "Tried deleting page %d - this is not allowed!", id);
    ASSERT(0);
  }

  if(id > MAX_VALID_PAGE){
    ESP_LOGE(TAG, "Tried deleting page %d - this is not allowed!", id);
    ASSERT(0); 
  }

  int offset = id*FLASH_PAGE_SIZE;
  if (flash_erase(flash_dev, offset, FLASH_PAGE_SIZE) != 0) {
    ESP_LOGE(TAG, "Failed to delete page %d ?!", id);
    ASSERT(0);
  } else { 
    ESP_LOGE(TAG, "Deleted page %d", id);
  }
}

// The flash layer has no idea if the BLE chunk
// was sent or not - the shim will be responsible 
// for that logic, the shim will inform the flash
// layer of the status of the BLE chunk transmit
void set_chunk_sent(uint32_t chunk){
  if (chunk > UPLOAD_CHUNKS_IN_PAGE){
    ESP_LOGE(TAG, "setting chunk (%d) out of range!", chunk);
  }
  
  ESP_LOGE(TAG, "Uploaded chunk(%d)!", chunk);
  uploaded[chunk] = true;

  // chunk is 0 indexed
  // on the final chunk sent, reset the logic
  if(chunk == UPLOAD_CHUNKS_IN_PAGE - 1){
    ESP_LOGI(TAG, "Finished uploading page %d", oldest_info.oldest_id);
    memset(uploaded, 0, sizeof(uploaded));
    erase_page(oldest_info.oldest_id_page);
    curr.total_valid_pages--;
    get_oldest_page();
  }
}

// returns:
//  UPLOADING_DONE         >if nothing left to upload
//  CHUNK_PROBLEM          >if there are problems
//  PAGE_FINISHED_UPLOAD   >if a page was fully uploaded (and internally erased)
//  CHUNK_VALID            >if there is something to push
//          -> data         = (data to upload)
//          -> ready_chunk  = chunk pending to be uploaded
int upload_incident_chunk(uint8_t ** data, uint32_t * ready_chunk){
  if (!ready_chunk){
    ESP_LOGE(TAG, "Error! null arg!");
    ASSERT(0);
  }

  if (oldest_info.oldest_id == MAX_VALID_ID){
    ESP_LOGI(TAG, "Nothing to upload!");
    return UPLOADING_DONE;
  }
  
  int chunk;
  for(chunk = 0; chunk < UPLOAD_CHUNKS_IN_PAGE; chunk++){
    if (uploaded[chunk]){
      continue;
    } 
    ESP_LOGI(TAG, "Chunk %d to be uploaded!", chunk);
    break;
  }
  
  // done uploading
  if(chunk == UPLOAD_CHUNKS_IN_PAGE){
    ESP_LOGI(TAG, "Finished uploading!"); 
    goto cleanup;
  }
  
  memset(flash_memblock, 0, UPLOAD_SIZE_CHUNK);
  int offset = oldest_info.oldest_id_page*FLASH_PAGE_SIZE + chunk*UPLOAD_SIZE_CHUNK; 
  ESP_LOGI(TAG, "uploading offset %d to cloud!", offset);
  if (flash_read(flash_dev, offset, flash_memblock, UPLOAD_SIZE_CHUNK) != 0) {
    ESP_LOGE(TAG, "Flash read of offset %u failed!", offset);
    return CHUNK_PROBLEM; 
  }
  #ifdef CONFIG_LOG
  print_hex(flash_memblock, UPLOAD_SIZE_CHUNK);
  #endif  

  // verify there is data in the page
  // it's possible we send up a partially valid packet,
  // we will let the backend deal w/ that
  flash_packet_t *test = (flash_packet_t*)flash_memblock;
  if (test->type == PAGE_HEADER_MAGIC_EMTPY){
    ESP_LOGI(TAG, "Chunk is empty, not uploading %d", chunk);
    goto cleanup;
  }   
  
  *data = flash_memblock;
  ESP_LOGE(TAG, "Ready to upload chunk %d", chunk);
  *ready_chunk = chunk;
  return CHUNK_VALID;

cleanup:
  memset(uploaded, 0, sizeof(uploaded));
  erase_page(oldest_info.oldest_id_page);
  curr.total_valid_pages--;
  get_oldest_page();
  return PAGE_FINISHED_UPLOAD;
}


// get the oldest page, if there 
// is no oldest page, that means
// the flash is empty
bool flash_is_empty(){
  get_oldest_page();
  if (oldest_info.oldest_id == MAX_VALID_ID){
    return true;
  }
  return false;
}

uint32_t get_age_oldest_page(){
  get_oldest_page();
  if (oldest_info.oldest_id == MAX_VALID_ID){
    return MAX_TIME;
  }
  flash_packet_t header;
  memset(&header, 0, sizeof(header));
  parse_header_packet(&header, oldest_info.oldest_id_page);
  if (header.type == PAGE_HEADER_MAGIC_EMTPY){
    ESP_LOGE(TAG, "Something wrong, page header was not written for page %d?", oldest_info.oldest_id_page);
    ASSERT(0);
  }  

  ESP_LOGI(TAG, "Oldest page is %u old", header.utc);
  return header.utc;
}

uint16_t get_total_valid_pages(){
  return curr.total_valid_pages;
}

void erase_incidence_data(){
  for(int i = 0; i < TOTAL_PAGES_FOR_INCIDENCE_DATA; i++){
    int offset =  OFFSET_TO_FIRST_INCIDENCE_DATA + FLASH_PAGE_SIZE*i;
    ESP_LOGI(TAG, "Erasing page offset %d", offset);
    if (flash_erase(flash_dev, offset, FLASH_PAGE_SIZE) != 0) {
      ESP_LOGE(TAG, "Flash erase failed!\n");
      ASSERT(0);
    }
  }
}

void set_config_data(int id){
    int offset =  FLASH_PAGE_SIZE*MAX_VALID_PAGE;
    if (flash_erase(flash_dev, offset, FLASH_PAGE_SIZE) != 0) {
      ESP_LOGE(TAG, "Flash erase failed!\n");
      ASSERT(0);
    }
    
    memset(flash_memblock, 0, 128);
    memcpy(flash_memblock, &id, 4);
    if (flash_write(flash_dev, offset, flash_memblock, 128) != 0) {
      ESP_LOGE(TAG, "Flash write failed!");
      return;
    }
}

int get_config_data(){
    memset(flash_memblock, 0, 128);
    int offset =  FLASH_PAGE_SIZE*MAX_VALID_PAGE;
    if (flash_read(flash_dev, offset,flash_memblock, 128) != 0) {
      ESP_LOGE(TAG, "Flash read failed!");
      return;
    }
    
    int ret;
    memcpy(&ret, flash_memblock, 4);
    return ret;
}


#ifdef TEST_MODE_FLASH 
#define FIRST_NON_IMAGE_PAGE FLASH_AREA_OFFSET(image_1)
static char test_buff[FLASH_PAGE_SIZE];

void erase_image_1(){
  for(int i = 0; i < TOTAL_PAGES_FOR_INCIDENCE_DATA; i++){
    int offset =  OFFSET_TO_FIRST_INCIDENCE_DATA + FLASH_PAGE_SIZE*i;
    if (flash_erase(flash_dev, offset, FLASH_PAGE_SIZE) != 0) {
      printf("Flash erase failed!\n");
      ASSERT(0);
    }
  }
}

void erase_image_small(){
  for(int i = 0; i < TOTAL_PAGES_FOR_INCIDENCE_DATA; i++){
    int offset =  OFFSET_TO_FIRST_INCIDENCE_DATA + FLASH_PAGE_SIZE*i;
    if (flash_erase(flash_dev, offset, FLASH_PAGE_SIZE) != 0) {
      ESP_LOGE(TAG, "Flash erase failed!\n");
      ASSERT(0);
    }
  }
}

void flash_test(){

set_current_time(1635451240);

//#define FOOBAR
#ifdef FOOBAR
  for(int i = 0; i < 50; i++){
    uint8_t * ptr;
    uint32_t chunk;
    if( 0 == upload_incident_chunk(ptr, &chunk) ){
      //logic (send BLE)

      set_chunk_sent(chunk);
    }
  }
  k_sleep(K_SECONDS(5));
#else
  k_sleep(K_SECONDS(5));
  erase_image_small();
  trace_packet_t foobar;
  
  for(int i = 0; i < 400 ; i++){
    write_packet(&foobar);
  }
  get_age_oldest_page();
  printf("total pages = %d", curr.total_valid_pages);
  k_sleep(K_SECONDS(100));
#endif 

/*

  printf("\nTest 1: Flash erase page at 0x%x\n", FIRST_NON_IMAGE_PAGE);
  if (flash_erase(flash_dev, FIRST_NON_IMAGE_PAGE, FLASH_PAGE_SIZE) != 0) {
    printf("Flash erase failed!\n");
  } else {
    printf("Flash erase succeeded!\n");
  }

  printf("\nTest 2: Flash write (word array 1)\n");
  if (flash_write(flash_dev, FIRST_NON_IMAGE_PAGE, test_buff, FLASH_PAGE_SIZE) != 0) {
      printf("Flash write failed!\n");
      return;
  }

  memset(test_buff, 0 ,FLASH_PAGE_SIZE);
  printf("Attempted to read 0x%x\n", 0);
  if (flash_read(flash_dev, FIRST_NON_IMAGE_PAGE, &test_buff, FLASH_PAGE_SIZE) != 0) {
      printf("   Flash read failed!\n");
      return;
  }
  memset(test_buff, 0 ,FLASH_PAGE_SIZE);
  printf("Attempted to read 0x%x\n", 0);
  if (flash_read(flash_dev, FIRST_NON_IMAGE_PAGE, &test_buff, FLASH_PAGE_SIZE) != 0) {
      printf("   Flash read failed!\n");
      return;
  }

*/
}
#endif 
