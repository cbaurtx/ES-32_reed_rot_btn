/**
 * @file    keypad_example_main.c
 * @author  Christof Baur
 * @date    09.04.2023
 * @version 0.9
 * @copyright   Christof Baur
 * @brief   ESP-32 library to interface with a rotary encoder and buttons
 *
 * License: MIT (see doc/LICENSE)
 */


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"

#include "reed_rot_btn.h"
void app_main(void)
{
    unsigned int key_code;
    static unsigned int key1_dly;
    esp_err_t err;

    reed_rot_btn_init();

    for(;;) {

      err = reed_rot_btn_read(&key_code, &key1_dly, CONFIG_ROT_KEY_TIMEOUT);
      if(err == ESP_OK)
        switch (key_code & 3) {
          case BTN_EVNT:
            printf("Button event: %x\n", (key_code & 0xfffc) >> 2);
            break;
          case ROT_EVNT:
            printf("Turn event:  %d\n", key_code >> 0x12 );
            break;
          default:
            printf("Invalid\n");

        }
      if(err == ESP_ERR_TIMEOUT)
        printf("Timed out\n");

      taskYIELD();
    }
}

