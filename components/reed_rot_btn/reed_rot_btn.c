/**
 * @file    reed_rot_btn.c
 * @author  Christof Baur
 * @date    09.04.2023
 * @version 0.9
 * @copyright Christof Baur
 * @brief   ESP32D library to interface with a dual pole reed contact, a mechanical rotary encoder and buttons
 *
 * License: MIT (see doc/LICENSE)
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include "esp_idf_version.h"
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

#include "ulp_reed_rot_btn.h"
#include "reed_rot_btn.h"
#include "sdkconfig.h"

#define AB_RTC_B_GPIO_NUM (CONFIG_AB_RTC_A_GPIO_NUM + 1)
#define AB_RTC_GPIO_MASK (3 << CONFIG_AB_RTC_A_GPIO_NUM)
#define IO_REED0_MASK (1 << CONFIG_REED0_RTC_GPIO_NUM)
#define IO_REED1_MASK (1 << CONFIG_REED1_RTC_GPIO_NUM)

static const char* TAG = "reed_rot_btn";

extern const uint8_t ulp_reed_rot_btn_bin_start[]
asm("_binary_ulp_reed_rot_btn_bin_start");
extern const uint8_t ulp_reed_rot_btn_bin_end[]
asm("_binary_ulp_reed_rot_btn_bin_end");

static void IRAM_ATTR reed_rot_btn_isr(void *);
static void init_ulp_prog(void);
static void start_ulp_prog(void);
static void init_rtcios(void);
static void set_wake_mask(void);

static void init_rtcio(unsigned int rtcio);


/* shared with ISR */
static TaskHandle_t volatile recv_task;
static uint32_t volatile reed_rot_btn_code;

/*  for speed calculation */
static uint32_t reed_time_prev;
static uint32_t reed_time_curr;
static uint32_t reed_time_delay;
static uint32_t time_curr;

static const uint8_t rtc2gpio_map[16] = {36, 37, 38, 39, 34, 35, 25, 26, 33, 32, 4, 0, 2, 15, 13};

/**
 * Initialize reed contact, rotary AB decoder and buttons
 *
 * @note Must be called from a task
 * @note Uses the ULP (loads the ULP program into RTC slow memory)
 * @note Configure reed IO, encoder AB IO and buttons IO using menuconfig
 * @note IOs must have an external pull-down resistor (2.2 Meg Ohms) and switchs to Vcc
 *
 */
void reed_rot_btn_init()
{
  /* register isr, enable interrupt */
  #if IDF_VER_MAJOR < 5
  ESP_ERROR_CHECK(rtc_isr_register(reed_rot_btn_isr, NULL, RTC_CNTL_SAR_INT_ST_M));
  // ESP_ERROR_CHECK(rtc_isr_register(reed_rot_btn_isr, NULL, RTC_CNTL_SAR_INT_ST_M));
  REG_SET_BIT(RTC_CNTL_INT_ENA_REG, RTC_CNTL_ULP_CP_INT_ENA_M);
  #else

  #endif
  init_ulp_prog();
  init_rtcios();
  recv_task = xTaskGetCurrentTaskHandle();
  set_wake_mask();
  reed_time_prev = xTaskGetTickCount();
  ulp_rot_cnt = 0x8000;
  start_ulp_prog();
  ESP_LOGD(TAG, "Initialized");
}

esp_err_t reed_rot_btn_read(unsigned int* p_key_code, unsigned int *reed_delay ,int timeout)
{
 recv_task = xTaskGetCurrentTaskHandle();

 if (xTaskNotifyWait(0x00, ULONG_MAX, p_key_code, timeout) == pdTRUE) {
   ESP_LOGD(TAG, "rot_debug = %x", ulp_rot_tmp & 0x0ffff);
   if ((*p_key_code & 0x0003) == REED_EVNT){
      // reed_time_curr = xTaskGetTickCount();
      ESP_LOGD(TAG, "reed_time_delay = %d", reed_time_delay);
      *reed_delay = reed_time_delay ;
      }
   return(ESP_OK);
   }
 else {
   // reed_time_curr = xTaskGetTickCount();
   ESP_LOGD(TAG, "Timeout\n");
   return(ESP_ERR_TIMEOUT);
   }
}


static void IRAM_ATTR reed_rot_btn_isr(void* arg)
/**
 * ULP interrupt service routine. ULP interrupt triggered when ULP executes 'wake' instruction
 * and the Xtensa is not sleeping.
 */
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    time_curr = xTaskGetTickCountFromISR();   // assign time as quickly as possible
    if ((ulp_evnt & 0x0003) == REED_EVNT) {
      reed_time_curr = time_curr;
      reed_time_delay = reed_time_curr - reed_time_prev;
      }
    reed_rot_btn_code = 0xffff & ulp_evnt;   // ulp variables are 32 bit, but only the lower 16 are valid
    reed_rot_btn_code |= (0xffff & ulp_btn) << 4;
    reed_rot_btn_code |= (0xffff & ulp_rot_cnt) << 16;
    ulp_btn = 0;
    ulp_evnt = 0;

    /* notify blocked task*/
    xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(recv_task, reed_rot_btn_code, eSetValueWithoutOverwrite, &xHigherPriorityTaskWoken);

    /* clear interruopt */
    WRITE_PERI_REG(RTC_CNTL_INT_CLR_REG, READ_PERI_REG(RTC_CNTL_INT_ST_REG));

    portYIELD_FROM_ISR();
}

static void init_ulp_prog(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_reed_rot_btn_bin_start,
            (ulp_reed_rot_btn_bin_end - ulp_reed_rot_btn_bin_start) / sizeof(uint32_t));
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
    ESP_LOGD(TAG, "Init Encoder A RTCIO %d\n", CONFIG_AB_RTC_A_GPIO_NUM);
    init_rtcio(CONFIG_AB_RTC_A_GPIO_NUM);
    ESP_LOGD(TAG, "Init Encoder B RTCIO %d\n", AB_RTC_B_GPIO_NUM);
    init_rtcio(AB_RTC_B_GPIO_NUM);

    /* Init RTCIOs for buttons */
    for(count=0;count<15;count++)
        if ((CONFIG_BTN_RTC_GPIO_MASK >> count) & 0x01) {
            ESP_LOGI(TAG, "Init Button RTCIO %d\n", count);
            init_rtcio(count);
            }

    /* RTCIO for reed0 and reed1*/
    ESP_LOGD(TAG, "Init Reed0 RTCIO %d\n", CONFIG_REED0_RTC_GPIO_NUM);
    init_rtcio(CONFIG_REED0_RTC_GPIO_NUM);
    ESP_LOGD(TAG, "Init Reed1 RTCIO %d\n", CONFIG_REED1_RTC_GPIO_NUM);
    init_rtcio(CONFIG_REED1_RTC_GPIO_NUM);

    #ifdef CONFIG_REED_DEBOUNCE_OUT
      ESP_LOGI(TAG, "Reed debounce out: true");
      ESP_LOGI(TAG, "Init Reed debounce RTCIO (out) %d\n", CONFIG_REED_DEBOUNCE_RTC_GPIO_NUM);
      init_rtcio(CONFIG_REED_DEBOUNCE_RTC_GPIO_NUM);
      rtc_gpio_set_direction(rtc2gpio_map[CONFIG_REED_DEBOUNCE_RTC_GPIO_NUM], RTC_GPIO_MODE_OUTPUT_ONLY);
      rtc_gpio_pullup_en(rtc2gpio_map[CONFIG_REED_DEBOUNCE_RTC_GPIO_NUM]);

    #endif

    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors.
     * GPIO12 may be pulled high to select flash voltage.
     * Note that gpio 15 is needed for the SD card => default do not isolate
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
      /* GPIO 2is needed for SD card so leave it as is by setting
       * CONFIG_ENABLE_GPIO_2_OUT to N (kconfig) */
      ESP_LOGD(TAG, "Configure GPIO 2 as output");
      /* rtc_gpio 2 as output (Debug LED) */
      rtc_gpio_init(GPIO_NUM_2);
      rtc_gpio_set_direction(GPIO_NUM_2, RTC_GPIO_MODE_OUTPUT_ONLY);
      rtc_gpio_pulldown_dis(GPIO_NUM_2);
      rtc_gpio_pullup_dis(GPIO_NUM_2);
    #endif
    ESP_LOGD(TAG, "RTCIO Mask 0x%x", (IO_REED0_MASK | IO_REED1_MASK | AB_RTC_GPIO_MASK | CONFIG_BTN_RTC_GPIO_MASK));
}

static void init_rtcio(unsigned int rtc_io)
{
 int gpio;

 gpio = rtc2gpio_map[rtc_io];

 ESP_LOGD(TAG, "init RTCIO%d = GPIO %d\n", rtc_io, gpio);

 rtc_gpio_init(gpio);                                    // after sleep io mode is RTC, but to be sure...
 rtc_gpio_set_direction(gpio, RTC_GPIO_MODE_INPUT_ONLY); // direction input (also needed for input only pins)
 rtc_gpio_hold_dis(gpio);
 rtc_gpio_pulldown_dis(gpio);                  // Some pins do not feature poll-up, pull-down, so never use
 rtc_gpio_pullup_dis(gpio);
}


static void deinit_rtcio(unsigned int rtc_io)
{
 int gpio;

 gpio = rtc2gpio_map[rtc_io];

 ESP_LOGD(TAG, "deinit GPIO %d\n", gpio);
 rtc_gpio_pulldown_dis(gpio);
 rtc_gpio_pullup_dis(gpio);
 // rtc_gpio_isolate(gpio);
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
