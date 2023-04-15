/**
 * @file    keypad_example_main.c
 * @author  Christof Baur
 * @date    09.04.2023
 * @version 0.92
 * @copyright   Christof Baur
 * @brief   ESP-32 library to interface with a rotary encoder and buttons
 *
 * License: MIT (see doc/LICENSE)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "reed_rot_btn.h"

static const char* TAG = "keypad_example_main.c";

void app_main(void) {
    unsigned int key_code;
    static unsigned int key1_dly;
    esp_err_t err;

    reed_rot_btn_init();

    for (;;) {
        err = reed_rot_btn_read(&key_code, &key1_dly, CONFIG_ROT_KEY_TIMEOUT);
        if (err == ESP_OK) switch (key_code & 3) {
                case REED_EVNT:
                    ESP_LOGI(TAG, "Reed event");
                    ESP_LOGI(TAG, "Reed delay %d", key1_dly);
                    break;
                case BTN_EVNT:
                    ESP_LOGI(TAG, "Button event: %x", (key_code & 0xfffc) >> 2);
                    break;
                case ROT_EVNT:
                    ESP_LOGI(TAG, "Turn event:  %d", key_code >> 0x10);
                    break;
                default:
                    ESP_LOGI(TAG, "Invalid");
            }
        if (err == ESP_ERR_TIMEOUT) ESP_LOGI(TAG, "Timed out\n");

        taskYIELD();
    }
}
