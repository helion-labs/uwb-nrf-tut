#pragma once
#include <stdio.h>
#include <power/reboot.h> // required for assert

#ifdef CONFIG_LOG
 #define ESP_LOGE(tag, format, ... )                                    \
   do {                                                                 \
     printf("%s: ", tag);                                               \
     printf(format, ##__VA_ARGS__);                                     \
     printf("\n");                                                      \
 }while(0)
#else 
  #define ESP_LOGE(tag, format, ... ) do { } while(0)
#endif   

#ifdef CONFIG_LOG
 #define ESP_LOGI(tag, format, ... )                                    \
   do {                                                                 \
     printf("%s: ", tag);                                               \
     printf(format, ##__VA_ARGS__);                                     \
     printf("\n");                                                      \
 }while(0)
#else 
  #define ESP_LOGI(tag, format, ... ) do { } while(0)
#endif   

#define ASSERT(x)                                                       \
    do {                                                                \
        if (!(x)) {                                                     \
            printf( "ASSERT! error %s %u\n", __FILE__, __LINE__);       \
            for (;;) {                                                  \
               sys_reboot(1);                                           \
            }                                                           \
        }                                                               \
    } while (0)


// Global project defines
#define DEBUG_CONFIG_DUMP_TO_BLE
