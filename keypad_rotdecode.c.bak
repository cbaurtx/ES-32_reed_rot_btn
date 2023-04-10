/**
 * @file    keypad_rotdecode.c
 * @author  Christof Baur
 * @date    13.04.2021
 * @version 0.1
 * @copyright Christof Baur
 * @brief   ESP-32 library to interface with a rotary encoder and buttons
 *
 * License: MIT
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "driver/rtc_io.h"
#include "driver/rtc_cntl.h"
#include "esp32/ulp.h"
#include "esp_err.h"
#include "esp_log.h"

#include "ulp_debounce_decode.h"
#include "keypad_rotdecode.h"
#include "sdkconfig.h"

#include "esp_timer.h"

#define  CONFIG_AB_RTC_B_GPIO_NUM  (CONFIG_AB_RTC_A_GPIO_NUM + 1)
#define CONFIG_AB_RTC_GPIO_MASK (3 << CONFIG_AB_RTC_A_GPIO_NUM)

static const char* TAG = "keypad_rotdecode";

extern const uint8_t ulp_debounce_decode_bin_start[]
asm("_binary_ulp_debounce_decode_bin_start");
extern const uint8_t ulp_debounce_decode_bin_end[]
asm("_binary_ulp_debounce_decode_bin_end");

static uint64_t key1_curr;
static uint64_t key1_last;

static void IRAM_ATTR key_rot_isr(void *);
static void init_ulp_prog(void);
static void start_ulp_prog(void);
static void init_rtcios(void);
static void set_wake_mask(void);

static void init_rtcio(unsigned int rtcio);


/* shared with ISR */
static TaskHandle_t volatile recv_task;
static unsigned int volatile key_rot_code;
static int volatile cnt_rot;

static const uint8_t rtc2gpio_map[16] = {36, 37, 38, 39, 34, 35, 25, 26, 33, 32, 4, 0, 2, 15, 13, 12};

/**
 * Initialize keypad and rotary AB decoder
 *
 * @note Must be called from a task so
 * @note Uses the ULP (loads the ULP program into RTC memory)
 * @note Configure key inputs and encoder AB inputs using menuconfig
 * @note Inputs must have an external pull-down resistor and switchs to GND
 *
 */
void key_rot_init()
{
  /* register isr, enable interrupt */
  ESP_ERROR_CHECK(rtc_isr_register(key_rot_isr, NULL, RTC_CNTL_SAR_INT_ST_M));
  REG_SET_BIT(RTC_CNTL_INT_ENA_REG, RTC_CNTL_ULP_CP_INT_ENA_M);
  init_ulp_prog();
  init_rtcios();
  init_rtcio(CONFIG_DEBOUNCE_RTCIO);
  recv_task = xTaskGetCurrentTaskHandle();
  set_wake_mask();
  start_ulp_prog();
  key1_last = esp_timer_get_time();
  cnt_rot = 0;
  ESP_LOGI(TAG, "Initialized");
}


esp_err_t key_rot_read(unsigned int* p_key_code, int *key1_period_p, int timeout)
{
 recv_task = xTaskGetCurrentTaskHandle();

 if (xTaskNotifyWait(0x00, ULONG_MAX, p_key_code, timeout) == pdTRUE) {
   if (*p_key_code == 5) {
      key1_curr = esp_timer_get_time();
      *key1_period_p = (int) ((key1_curr - key1_last) / 1000);
      key1_last= key1_curr;
      }
   return(ESP_OK);
 }
 else {
   *key1_period_p = INT_MAX;
   return(ESP_ERR_TIMEOUT);
   }
}


static void IRAM_ATTR key_rot_isr(void* arg)
/**
 * ULP interrupt service routine. ULP interrupt triggered when ULP executes 'wake' instruction
 * and the Xtensa is not sleeping.
 */
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    cnt_rot += (short int)ulp_rot_st;
    if (cnt_rot < 0)
        cnt_rot = CONFIG_ROT_CNT_MAX;
    if (cnt_rot > CONFIG_ROT_CNT_MAX)
        cnt_rot = 0;

    key_rot_code = 0x0003 & ulp_evnt;
    key_rot_code |= (0xffff & ulp_btn) << 2;
    key_rot_code |= (0xffff & cnt_rot) << 16;

    ulp_evnt = 0;
    ulp_rot_st = 0;
    ulp_btn = 0;

    /* notify blocked task*/
    xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(recv_task, key_rot_code, eSetValueWithoutOverwrite, &xHigherPriorityTaskWoken);

    /* clear interruopt */
    WRITE_PERI_REG(RTC_CNTL_INT_CLR_REG, READ_PERI_REG(RTC_CNTL_INT_ST_REG));

    portYIELD_FROM_ISR();
}

static void init_ulp_prog(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_debounce_decode_bin_start,
            (ulp_debounce_decode_bin_end - ulp_debounce_decode_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* note: ULP variables are initialized by the ULP
     * Each 'ulp_xyz' variable corresponds to 'xyz' variable in the ULP program.
     * Need to include 'ulp_main.h' for this to work
    */
}


static void start_ulp_prog(void)
{
    /* Start the program */
    ESP_ERROR_CHECK(ulp_run(&ulp_entry - RTC_SLOW_MEM));
    ESP_LOGI(TAG, "ULP started");
}

static void init_rtcios(void)
{
    int count;

    /* Init RTCIOs for AB encoder signals */
    ESP_LOGD(TAG, "Init RTCIO %d\n", CONFIG_AB_RTC_A_GPIO_NUM);
    init_rtcio(CONFIG_AB_RTC_A_GPIO_NUM);
    ESP_LOGD(TAG, "Init RTCIO %d\n", CONFIG_AB_RTC_B_GPIO_NUM);
    init_rtcio(CONFIG_AB_RTC_B_GPIO_NUM);

    /* Init RTCIOs for buttons */
    for(count=0;count<15;count++)
        if ((CONFIG_BTN_RTC_GPIO_MASK >> count) & 0x01) {
            ESP_LOGI(TAG, "Init RTCIO %d\n", count);
            init_rtcio(count);
            }

    /* RTCIO to debounce RTC_IO00*/
    init_rtcio(CONFIG_DEBOUNCE_RTCIO);

    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors.
     * GPIO12 may be pulled high to select flash voltage.
     * Note that gpio 15 is needed for the SD card
     */
    #if CONFIG_ISOLATE_GPIO_12
      ESP_LOGD(TAG, "Isolate GPIO 12");
      rtc_gpio_isolate(GPIO_NUM_12);
    #endif

    #if CONFIG_ISOLATE_GPIO_15
      ESP_LOGD(TAG, "Isolate GPIO 15");
      rtc_gpio_isolate(GPIO_NUM_15);
    #endif

    #if CONFIG_ENABLE_GPIO_2_OUT
      /* GPIO 2is needed for SD card */
      ESP_LOGD(TAG, "Configure GPIO 2 as output");
      rtc_gpio 2 as output (Debug LED) */
      rtc_gpio_init(GPIO_NUM_2);
      rtc_gpio_set_direction(GPIO_NUM_2, RTC_GPIO_MODE_OUTPUT_ONLY);
      rtc_gpio_pulldown_dis(GPIO_NUM_2);
      rtc_gpio_pullup_dis(GPIO_NUM_2);
    #endif
    ESP_LOGD(TAG, "RTCIO Mask 0x%x", (CONFIG_AB_RTC_GPIO_MASK | CONFIG_BTN_RTC_GPIO_MASK));
}

static void init_rtcio(unsigned int rtc_io)
{
 int gpio;

 gpio = rtc2gpio_map[rtc_io];

 ESP_LOGD(TAG, "init GPIO %d\n", gpio);

 rtc_gpio_init(gpio);                     // after sleep io mode is RTC, but to be sure...
 rtc_gpio_set_direction(gpio, RTC_GPIO_MODE_INPUT_ONLY); // direction input (also needed for input only pins)
 rtc_gpio_pulldown_en(gpio);
 rtc_gpio_pullup_dis(gpio);
}


static void deinit_rtcio(unsigned int rtc_io)
{
 int gpio;

 gpio = rtc2gpio_map[rtc_io];

 ESP_LOGD(TAG, "deinit GPIO %d\n", gpio);
 rtc_gpio_pulldown_dis(gpio);
 rtc_gpio_pullup_dis(gpio);
 rtc_gpio_isolate(gpio);
}


static void set_wake_mask(void)
{
    uint64_t  gpio_mask;
    int count;

    gpio_mask = 0;

    for(count=0;count<15;count++)
        if ((CONFIG_RTC_GPIO_WAKE_MASK >> count) & 1)
            gpio_mask |= (1ULL << (uint64_t)rtc2gpio_map[count]);

    ESP_LOGD(TAG, "Deep sleep GPIO mask = %Lx", gpio_mask);
    esp_sleep_enable_ext1_wakeup(gpio_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
}
