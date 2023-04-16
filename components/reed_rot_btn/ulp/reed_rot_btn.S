/**
 * @file    reed_rot_btn.S
 * @author  Christof Baur
 * @date    15.04.2023
 * @version 0.92
 * @copyright   Christof Baur
 * @brief  ULP polls a double pole reed contact, a mechanical rotary AB encoder and push buttons
 * All signals are software debounced
 * Constants are defined with Kconfig
 *
 * License: MIT (see doc/LICENSE)
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

#include "sdkconfig.h"

/**
 * ULP register usage:
 * r0 temporary
 * r1 current value
 * r2 previous value
 * r3 memory address / temporary
 * stcnt number of reed triggers. Rolls over at REED_DIV
 *
 * Variables
 * evnt                     event code: 1 = reed triggered REED_DIV times; 2 = rotated; 3 = button pressed
 * wait_reed_cnt   timeout count reed; ==0: not waiting, ==1: timeout expired, >1: waiting
 * wait_rot_cnt      timeout count rotation; <0: not waiting, ==0: timeout expired, >0: waiting
 * wait_btn_cnt     timeout count buttons; <0: not waiting, ==0: timeout expired, >0: waiting
 * rot_cnt                counter for rotation
 * rot_tmp              temporary storage of lookup table index
 * btn                      Each bit corresponds to a button
 *
 * Configurable (Kconfig) constants
 * CONFIG_WAIT_REED_SET        wait value for reed debounce
 * CONFIG_WAIT_ROT_SET          wait value for rotation debounce
 * CONFIG_WAIT_BTN_SET          wait value for buttons debounce
 * CONFIG_REED0_RTC_GPIO_NUM   number of RTC-IO with reed normaly open
 * CONFIG_REED1_RTC_GPIO_NUM   number of RTC-IO with reed normaly closed
 * CONFIG_REED_DEBOUNCE_OUT    output debounced signal to RTCIO
 * CONFIG_REED_DEBOUNCE_RTC_GPIO_NUM  RTC_GPIO to use for reed debounce out
 * CONFIG_AB_RTC_A_GPIO_NUM     IO number of encoder A signal
 *                                                                B signal must be next higher RTC-IO
 * CONFIG_BTN_RTC_GPIO_MASK     mask for RTC-IO push buttons
 * REED_DIV                                             divisior for reed counts
 */

#define REED_EVNT 1
#define ROT_EVNT 2
#define BTN_EVNT 3

#define IO_ROT_MASK (3 << CONFIG_AB_RTC_A_GPIO_NUM)
#define IO_REED0_MASK (1 << CONFIG_REED0_RTC_GPIO_NUM)
#define IO_REED1_MASK (1 << CONFIG_REED1_RTC_GPIO_NUM)

/* Define variables, which go into .bss section (zero-initialized data) */
  .bss

  /* global variabled, can be accessed by Xtensa cores */
        .global evnt
  evnt: .long 0
        .global rot_cnt
  rot_cnt: .long 0
        .global btn
  btn: .long 0

/* local variables, ULP access only */
  rot_tmp: .long 0
  wait_reed_cnt: .long 0
  wait_rot_cnt: .long 0
  wait_btn_cnt: .long 0

/* Code goes into .text section */
    .text

/* Look-up table to convert from AB old and AB new to counting ticks up or down
     1 count up
     2 count down
     0 indicates no change as no or an invalid transition occured */
  .global st_tbl
st_tbl:            // new old
    .long 0        // 00   00
    .long 1        // 00   01
    .long 2        // 00   11
    .long 0        // 01   00
    .long 2        // 01   01
    .long 0        // 01   10
    .long 0        // 01   11
    .long 1        // 10   00
    .long 1        // 10   01
    .long 0        // 10   10
    .long 0        // 10   11
    .long 2        // 11   00
    .long 0        // 11   01
    .long 2        // 11   10
    .long 1        // 11   11
    .long 0

    .global entry
entry:
    READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16)                                // prev_io = read IOs
    and r2, r0, (IO_REED0_MASK | IO_REED1_MASK | IO_ROT_MASK | CONFIG_BTN_RTC_GPIO_MASK)  // mask unused IOs
    stage_rst

loop_forever:
    /* Read the value of lower 16 RTC IOs into R0 */
    READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16)

    and r0, r0, (IO_REED0_MASK | IO_REED1_MASK | IO_ROT_MASK | CONFIG_BTN_RTC_GPIO_MASK)   // mask unused IOs
    move r1, r0                           // current IO = read IOs

    /* any IO changed? */
    or r0, r1, r2                           // r0 = r1 xor r2   = (r1 or r2) - (r1 and r2)
    and r3, r1, r2
    sub r0, r0, r3
    jump no_change, eq           // no IO changed

change:                                      // at least one IO changed
    move r3, r0                            // use r3 as tmp: r3 = r1 xor r2 (which still is in R0)

    and r0, r3, IO_REED0_MASK    // r0 = (current_io xor prev_io) and mask_reed0
    and r0, r0, r1                       // raising?
    jumpr reed0_raising, 1, ge

    and r0, r3, IO_REED1_MASK    // r0 = (current_io xor prev_io) and mask_reed1
    and r0, r0, r1
    jumpr reed1_raising, 1, ge

    and r0, r3, IO_ROT_MASK      // mask: rot
    jumpr rot_changed, 1, ge

    and r0, r3, CONFIG_BTN_RTC_GPIO_MASK     // mask: buttons
    and r0, r0, r1               // raising?
    jumpr btn_raising, 1, ge

    // must be reed or btn falling
    move r2, r1                  // prev_io = curr_io
    and r0, r3, CONFIG_BTN_RTC_GPIO_MASK     // mask: buttons
    jumpr btn_falling, 1, ge

    // reed0 or reed 1 falling
    and r0, r3, IO_REED1_MASK
    jumpr end_change, 1,  ge   // reed1 falling, do nothing

    move r3, wait_reed_cnt                   // set reed wait
    move r0, CONFIG_WAIT_REED_SET
    st r0, r3, 0
    jump end_change

btn_falling:
    move r3, wait_btn_cnt                   // set btn wait
    move r0, CONFIG_WAIT_BTN_SET
    st r0, r3, 0

end_change:
    jump loop_forever                        // invalid IO: do nothing

reed0_raising:
    move r3, wait_reed_cnt                   // check if waiting (debounce)
    ld r0, r3, 0
    jumpr end_reed0_raising, 1, ge           // waiting, reed0 bounced: do nothing
    move r0, CONFIG_WAIT_REED_SET            // set wait
    st r0, r3, 0
    /* for debugging send debounced signal to output*/
    #ifdef CONFIG_REED_DEBOUNCE_OUT
      WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + CONFIG_REED_DEBOUNCE_RTC_GPIO_NUM, 1, 1)
    #endif

    stage_inc 1
    jumps loop_forever, CONFIG_REED_DIV ,lt    // jum when reed counter not at max value
    move r3, evnt
    move r0, REED_EVNT
    st r0, r3, 0
    wake                                        // interrupt Xtensa
    stage_rst                                   // reed counter at max: reset counter

end_reed0_raising:
    move r2, r1                              // prev_io = curr_io
    jump loop_forever

reed1_raising:
    move r3, wait_reed_cnt
    ld r0, r3, 0
    jumpr end_reed1_raising, 1, lt           // not waiting, ignore
    move r0, 1                               // set reed wait counter to expire next
    st r0, r3, 0
end_reed1_raising:
    move r2, r1                              // prev_io = curr_io
    jump loop_forever

btn_raising:
    move r3, wait_btn_cnt
    ld r0, r3, 0
    jumpr end_btn_raising, 1, ge             // wait_btn_cnt > 0: button bounce
btn_pressed:
    and r0, r1, CONFIG_BTN_RTC_GPIO_MASK     // mask: buttons
    move r3, btn
    st r0, r3, 0
    move r3, wait_btn_cnt                    // set btn wait
    move r0, CONFIG_WAIT_BTN_SET
    st r0, r3, 0
    move r3, evnt                            // event = BTN_EVNT
    move r0, BTN_EVNT
    st r0, r3, 0
    wake                                     // interrupt Xtensa

end_btn_raising:
    move r2, r1                              // prev_io = curr_io
    jump loop_forever

rot_changed:
    rsh r0, r1, CONFIG_AB_RTC_A_GPIO_NUM - 2   // r0 bit 2 and 3 = curr_IO
    and r0, r0, 0x000c                       // mask rot
    rsh r3, r2, CONFIG_AB_RTC_A_GPIO_NUM     // r0 bit 0 and 1 = prev_IO
    and r3, r3, 0x0003                       // mask rot
    or r3, r0, r3

    move r0, rot_tmp
    st r3, r0, 0
    add r3, r3, st_tbl           // table look-up
    ld r0, r3, 0
    jumpr dec_rot_raise, 1, EQ   // r0 == 1 => rot_cnt--
    jumpr inc_rot_raise, 2, EQ   // r0 == 2 => rot_cnt++
    jump end_rot_raising         // invalid transition: ignore

inc_rot_raise:                 // rot_cnt++
    move r3,  rot_tmp
    ld r0, r3, 0
    jumpr inc_rot, 0x0b, EQ                // r0 == 0x0b; only detented positions
    jump end_rot_raising

inc_rot:
    move r3, rot_cnt
    ld r0, r3, 0
    add r0, r0, 1
    st r0, r3, 0
    jump finalyze_rot

dec_rot_raise:                 // rot_cnt--
    move r3, rot_tmp
    ld r0, r3, 0
    jumpr dec_rot, 0x07, EQ    // r0 == 0x07;  ; only detented positions
    jump end_rot_decreasing

dec_rot:
    move r3, rot_cnt
    ld r0, r3, 0
    sub r0, r0, 1
    st r0, r3, 0
    jump finalyze_rot

finalyze_rot:
    move r3, wait_rot_cnt        // set rot wait
    move r0, CONFIG_WAIT_ROT_SET
    st r0, r3, 0
end_rot_raising:
   move r2, r1                  // prev_io = curr_io
  jump loop_forever

end_rot_decreasing:
    move r2, r1                  // prev_io = curr_io
    jump loop_forever

no_change:
wait_reed:
    move r3, wait_reed_cnt
    ld r0, r3, 0
    jumpr wait_btn, 1, lt    // wait_reed_cnt was 0 => do nothing
    sub r0, r0, 1                 // count down
    st r0, r3, 0
    jumpr wait_reed_end, 1, ge    // wait not over
wait_reed_exp:
    #ifdef CONFIG_REED_DEBOUNCE_OUT
        WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + CONFIG_REED_DEBOUNCE_RTC_GPIO_NUM, 1, 0)
    #endif
jump wait_reed_end

wait_btn:
    move r3, wait_btn_cnt
    ld r0, r3, 0
    jumpr wait_rot, 1, lt        // wait_btn_cnt was 0 => do nothing
    sub r0, r0, 1                // count down wait
    st r0, r3, 0

wait_reed_end:
    jump loop_forever

wait_btn_end:
    jumpr wait_rot, 1, ge        // wait_btn_cnt is not 0 => do nothing
    #ifdef CONFIG_REED_DEBOUNCE_OUT
        WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + CONFIG_REED_DEBOUNCE_RTC_GPIO_NUM, 1, 0)
    #endif

wait_rot:
    move r3, wait_rot_cnt
    ld r0, r3, 0
    jumpr wait_rot_end, 1, lt    // wait_rot_cnt was 0 => do nothing
    sub r0, r0, 1                // count down wait
    st r0, r3, 0
    jumpr wait_rot_end, 1, ge    // wait not over
    move r3, evnt
    move r0, ROT_EVNT
    st r0, r3, 0
    wake                         // interrupt Xtensa
wait_rot_end:
    jump loop_forever
