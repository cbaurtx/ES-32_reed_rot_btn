/**
 * @file    keypad_rotdecode.h
 * @author  Christof Baur
 * @date    13.04.2021
 * @version 0.1
 * @copyright   Christof Baur
 * @brief  ULP polls keys of the rotary AB encoder as fast as possible until a signal changes
 * After a debounce wait time the signals are decoded.
 * Constants are defined with Kconfig
 *
 * License: MIT
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
 * r4 memory address
 *
 * Variables
 *
 * Constants (from Kconfig)
 *
 */




/* CONFIG_AB_RTC_A_GPIO_NUM holds the RTCIOI number of encoder signal A. Encoder signal
 * B must be conneted to the next higher RTCIIO
 */
#define CONFIG_AB_RTC_GPIO_MASK (3 << CONFIG_AB_RTC_A_GPIO_NUM)

  /* Define variables, which go into .bss section (zero-initialized data) */
  .bss

  /* Each bit corresponds to a button */
  .global btn
btn:
  .long 0

  /* io before botton was pressed or released or the encoder changed */
  .global prev_io
prev_io:
  .long 0

  /* current io values */
  .global curr_io
curr_io:
  .long 0

  /* ab encoder signals, previous and current. Bits 0-1 current; 2-3 previous */
  .global rot_ab
rot_ab:
  .long 0

  /* 0: no change; +1 rot cw; -1 rot ccw */
  .global rot_st
rot_st:
  .long 0

  /* Event 1 = btn pressed, 2 = btn released, 3 = rotated */
  .global evnt
evnt:
  .long 0

  /* Code goes into .text section */
  .text

  /* Look-up table to convert from AB old and new to counting ticks up or down
     0 indicates no change as an invalid transition occured */
  .global st_tbl
st_tbl:
  .long 0
  .long 1
  .long -1
  .long 0
  .long -1
  .long 0
  .long 0
  .long 1
  .long 1
  .long 0
  .long 0
  .long -1
  .long 0
  .long -1
  .long 1
  .long 0

  .global entry

entry:

  /* establish initial values */
  READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16)
  and r0, r0, (CONFIG_AB_RTC_GPIO_MASK | CONFIG_BTN_RTC_GPIO_MASK)

  move r2, r0
  move r3, prev_io
  st r0, r3, 0

  and r0, r0, CONFIG_AB_RTC_GPIO_MASK
  rsh r0, r0, CONFIG_AB_RTC_A_GPIO_NUM
  move r3, rot_ab
  st r0, r3, 0

loop_forever:
  /* Read the value of lower 16 RTC IOs into R0 */
  READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16)

  /* mask unused ios */
  and r0, r0, (CONFIG_AB_RTC_GPIO_MASK | CONFIG_BTN_RTC_GPIO_MASK)
  move r1, r0

  /* io changed ? */
  sub r0, r1, r2
  jump loop_forever, EQ

  /* store in variable io */
  move r3, curr_io
  st r1, r3, 0

  /* if changed IO was RTC_IO00 wait until CONFIG_DEBOUNCE_RTCIO changes to 1 */
  and r0, r1, 1
  jump debounce_wait, EQ
debounce_rtcio:
  /* Read again the values into R0 */
  READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16)
  and r0, r0, (1 << CONFIG_DEBOUNCE_RTCIO)
  jump debounce_rtcio, EQ
  wait CONFIG_RTCIO0_DEBOUNCE_WAIT
  jump io_chg

debounce_wait:
  /* else wait for bouncing to stop usee twice to extend range */
  wait CONFIG_BTN_DEBOUNCE_WAIT
  wait CONFIG_BTN_DEBOUNCE_WAIT

  /* Read again the values into R0 */
  READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16)

  /* mask unused ios */
  and r0, r0, (CONFIG_AB_RTC_GPIO_MASK | CONFIG_BTN_RTC_GPIO_MASK)
  sub r0, r0, r1

  /* io changed (no bounce and no transient) */
  jump io_chg, EQ
  jump loop_forever

io_chg:
  move r3, prev_io
  ld r2, r3, 0
  and r2, r2, CONFIG_BTN_RTC_GPIO_MASK

  move r3, curr_io
  ld r1, r3, 0
  and r1, r1, CONFIG_BTN_RTC_GPIO_MASK

  sub r0, r1, r2
  jump rot_enc, EQ
  jump btn_chg

rot_enc:
  /* encoder AB changed */
  move r3, evnt
  move r0, 3
  st r0, r3, 0

  /* decode AB */
  move r3, curr_io
  ld r1, r3, 0
  rsh r1, r1, CONFIG_AB_RTC_A_GPIO_NUM
  and r1, r1, 3

  move r3, rot_ab
  ld r0, r3, 0
  lsh r0, r0, 2
  or r0, r0, r1
  and r0, r0, 0x000f

  /* table look-up */
  add r1, r0, st_tbl
  ld r0, r1, 0

  /* store in rot_st */
  move r3, rot_st
  st r0, r3, 0

  /* store current ab as previous ab */
  move r3, curr_io
  ld r0, r3, 0

  rsh r0, r0, CONFIG_AB_RTC_A_GPIO_NUM
  and r0, r0, 3

  move r3, rot_ab
  st r0, r3, 0

  /* restore r1 and r2 */
  move r3, curr_io
  ld r1, r3, 0
  move r2, r1

  move r3, prev_io
  st r1, r3, 0

  /* trigger ULP int to notify Xtensa */
  wake

  jump loop_forever

btn_chg:
  move r3, curr_io
  ld r1, r3, 0
  and r1, r1, CONFIG_BTN_RTC_GPIO_MASK
  move r3, prev_io
  ld r2, r3, 0
  and r2, r2, CONFIG_BTN_RTC_GPIO_MASK

  sub r0, r1, r2
  jump h_to_l, OV

  /* debug LED on */
  /* WRITE_RTC_REG(RTC_GPIO_OUT_REG,RTC_GPIO_OUT_DATA_S+12,1,1) */

  move r3, evnt
  move r0, 1
  st r0, r3, 0
  jump cont_1

h_to_l:
  /* debug LED off */
  /* WRITE_RTC_REG(RTC_GPIO_OUT_REG,RTC_GPIO_OUT_DATA_S+12,1,0) */
  move r3, evnt
  move r0, 2
  st r0, r3, 0

  move r3, btn
  st r2, r3, 0
  jump cont_2

cont_1:
  /* store to variable btn */
  move r3, btn
  st r1, r3, 0

cont_2:
  /* restore r1 */
  move r3, curr_io
  ld r1, r3, 0

  /* previous value = current value */
  move r2, r1
  move r3, prev_io
  st r2, r3, 0

  /* trigger ULP interrupt to notify Xtensa */
  wake

  jump loop_forever
