/**
 * @file    keypad_rotdecode.h
 * @author  Christof Baur
 * @date    09.04.2023
 * @version 0.9
 * @copyright   Christof Baur
 * @brief   ESP-32 library to interface with a rotary encoder and buttons
 *
 * License: MIT (see doc/LICENSE)
 */

#ifndef KEYPAD_ROTDECODE_H
#define KEYPAD_ROTDECODE_H

#include "esp_err.h"

#define REED_EVNT 1
#define ROT_EVNT  2
#define BTN_EVNT  3


/**
 * Initialize reed, rotary AB encoder and buttons
 *
 * @note Uses the ULP (loads the ULP program into slow RTC memory)
 * @note Configure reed inputs, key inputs and encoder AB inputs using menuconfig
 * @note Inputs must have an external pull-down resistor and switches connect to Vcc
 *
 * @return
 *      - ESP_OK on success
 */
void reed_rot_btn_init(void);

/**
 * Blocking read
 * Register the currently running task as task waiting for reed, rot& key inputs
 *
 * @note Only a single task may be registered as the code is not reantrant capcable
 *
 * @return
 *      - ESP_OK            success
 *      - ESP_ERR_TIMEOUT   timeout
 */
esp_err_t reed_rot_btn_read(unsigned int *key_code_p, unsigned int *reed_delay ,int timeout);

#endif
