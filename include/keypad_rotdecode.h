/**
 * @file    keypad_rotdecode.h
 * @author  Christof Baur
 * @date    09.04.2023
 * @version 0.9
 * @copyright   Christof Baur
 * @brief   ESP-32 library to interface with a rotary encoder and buttons
 *
 * License: MIT  (see doc/LICENSE)
 */

#ifndef KEYPAD_ROTDECODE_H
#define KEYPAD_ROTDECODE_H

#include "esp_err.h"

#define EVNT_BTN_PRSS 1
#define EVNT_BTN_RELS 2
#define EVNT_ROT 3

/**
 * Initialize keypad and rotary AB decoder
 *
 * @note Uses the ULP (loads the ULP program into RTC memory)
 * @note Configure key inputs and encoder AB inputs using menuconfig
 * @note Inputs must have an external pull-down resistor and switchs to GND
 *
 * @param host  pointer to structure defining host controller
 * @param out_card  pointer to structure which will receive information
 *                  about the card when the function completes
 * @return
 *      - ESP_OK on success
 */
void key_rot_init(void);

/**
 * Blocking read
 * Register the currently running task as task waiting for key inputs
 *
 * @note Only a single task may be registered as the code is not reantrant capcable
 *
 * @return
 *      - ESP_OK        success
 *      - ESP_ERR_TIMEOUT   timeout
 */
esp_err_t key_rot_read(unsigned int *key_code_p, int *key1_period_p, int timeout);

#endif
