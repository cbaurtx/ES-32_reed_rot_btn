/**
 * @file    reed_rot_btn.s
 * @author  Christof Baur
 * @date    21.11.2021
 * @version 0.2
 * @copyright   Christof Baur
 * @brief  ULP polls a double pole reed contact, a mechanical rotary AB encoder and push buttons
 * All signals are debounced
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
 * r3 memory address
 * stcnt number of reed triggers. Rolls over at REED_DIV
 *
 * Variables
 * evnt           event code: 1 = reed triggered REED_DIV times; 2 = rotated; 3 = button pressed
 * tout_reed_cnt  timeout count reed; ==0: not waiting, ==1: timout expired, >1: waiting
 * tout_rot_cnt   timeout count rotation; <0: not waiting, ==0: timout expired, >0: waiting
 * tout_btn_cnt   timeout count buttons; <0: not waiting, ==0: timout expired, >0: waiting
 * rot_cnt        counter for rotation with ROT_STATES number of states
 * btn            Each bit corresponds to a button
 *
 * Constants (from Kconfig)
 * CONFIG_TOUT_REED_SET  timeout value for reed
 * CONFIG_TOUT_ROT_SET   timeout value for rotation
 * CONFIG_TOUT_BTN_SET   timout value for buttons
 * CONFIG_IO_REED0_MASK  mask for RTC-IO with reed normaly open
 * CONFIG_IO_REED1_MASK  mask for RTC-IO with reed normaly closed
 * CONFIG_AB_RTC_A_GPIO_NUM IO    number of encoder A signal
 *                                B signal must be next higher RTC-IO
 * CONFIG_IO_BTN_MASK    mask for RTC-IO for push buttons
 * REED_DIV              divisior for reed counts
 */

#define REED_EVNT 1
#define ROT_EVNT 2
#define BTN_EVNT 3

#define IO_ROT_MASK (3 << CONFIG_AB_RTC_A_GPIO_NUM)

  /* Define variables, which go into .bss section (zero-initialized data) */
  .bss
        .global evnt
  evnt: .long 0
        .global tout_reed_cnt
  tout_reed_cnt: .long 0
        .global tout_rot_cnt
  tout_rot_cnt: .long 0
        .global tout_btn_cnt
  tout_btn_cnt: .long 0
        .global rot_cnt
  rot_cnt: .long 0
        .global btn
  btn: .long 0

  /* Code goes into .text section */
  .text

  /* Look-up table to convert from AB old and new to counting ticks up or down
     1 count up
     2 count down
     0 indicates no change as no or an invalid transition occured */
  .global st_tbl
st_tbl:
  .long 0
  .long 1
  .long 2
  .long 0
  .long 2
  .long 0
  .long 0
  .long 1
  .long 1
  .long 0
  .long 0
  .long 2
  .long 0
  .long 2
  .long 1
  .long 0

  .global entry

entry:
  /* variables need to be initialized by Xtensa before starting the ULP */
  move r2, 0      ; previous IO = 0
  stage_rst

loop_forever:
  /* Read the value of lower 16 RTC IOs into R0 */
  READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16)

  /* mask unused ios */
  and r0, r0, (CONFIG_IO_REED0_MASK | CONFIG_IO_REED1_MASK | IO_ROT_MASK | CONFIG_IO_BTN_MASK)
  move r1, r0         ; to current IO
  /* any io changed? */
  sub r0, r1, r2
  jump no_change, eq      ; no IO changed
  jump falling_edge, ov   ;IO(s) changed from 1 to 0 => falling edge

raising_edge:
  move r2, r1             ; previous IO = current IO
  and r0, r1, CONFIG_IO_REED0_MASK    ; mask: reed0
  jumpr reed0_raising, 1, ge
  move r0, r1
  and r0, r1, CONFIG_IO_REED1_MASK    ; mask: reed1
  jumpr reed1_raising, 1, ge
  move r0, r1
  and r0, r1, IO_ROT_MASK             ; mask: rotary encoder
  jumpr rot_raising, 1, ge
  move r0, r1
  and r0, r1, CONFIG_IO_BTN_MASK      ; mask: buttons
  jumpr btn_raising, 1, ge
  jump loop_forever                   ; invalid, we never should get here

reed0_raising:
  move r3, tout_reed_cnt
  ld r0, r3, 0
  jumpr end_reed0_raising 1, ge  ;timout active => reed0 bounce
  move r0, CONFIG_TOUT_REED_SET
  st r0, r3, 0
end_reed0_raising:
  jump loop_forever

reed1_raising:
  move r3, tout_reed_cnt
  ld r0, r3, 0
  jumpr end_reed1_raising 1, lt   ;not waiting, ignore
  move r0, 1                      ;set reed0 timeout counter to expire next
  st r0, r3, 0
end_reed1_raising:
  jump loop_forever

btn_raising:
  move r3, tout_btn_cnt
  jumpr end_btn_raising, 1, ge        ; tout_btn_cnt > 0: button bounce
btn_pressed:
  move r0, r1
  and r0, r1, CONFIG_IO_BTN_MASK      ; mask: buttons
  move r3, btn
  st r0, r3, 0
  move r3, tout_btn_cnt
  move r0, CONFIG_TOUT_BTN_SET
  st r0, r3, 0
  move r3, evnt
  move r0, BTN_EVNT
  st r0, r3, 0
  wake
end_btn_raising:
  jump loop_forever

rot_raising:
  rsh r0, r1, CONFIG_AB_RTC_A_GPIO_NUM
  lsh r0, r0, 2
  rsh r3, r2, CONFIG_AB_RTC_A_GPIO_NUM
  or r3, r0, r3
  and r3, r3, 0x000f
  add r3, r3, st_tbl
  ld r0, r3, 0                 ; table look-up
  move r3, rot_cnt
  jumpr dec_rot_raise, 1, EQ   ;r0 == 1 => rot_cnt--
  jumpr end_rot_raising, 0, EQ ;r0 == 0 => invalid transition: ignore
inc_rot_raise:                 ;r0 == 2 => rot_cnt++
  ld r0, r3, 0
  add r0, r0, 1
  st r0, r3, 0
  jump finalyze_rot
dec_rot_raise:
  ld r0, r3, 0
  sub r0, r0, 1
  st r0, r3, 0
  jump finalyze_rot
end_rot_raising:
  jump loop_forever

falling_edge:                ;we always and only check for change of encoder AB.
  rsh r0, r1, CONFIG_AB_RTC_A_GPIO_NUM
  lsh r0, r0, 2
  rsh r3, r2, CONFIG_AB_RTC_A_GPIO_NUM
  or r3, r0, r3
  and r3, r3, 0x000f
  add r3, r3, st_tbl
  ld r0, r3, 0
  move r3, rot_cnt
  jumpr dec_rot_fall, 2, GE     ; lookup == 2: rot_cnt--
  jumpr end_rot, 1, LT          ; lookup == 0: skip
inc_rot_fall:                   ; lookup == 1: rot_cnt++
  ld r0, r3, 0
  add r0, r0, 1
  st r0, r3, 0
  jump finalyze_rot:
dec_rot_fall:
  ld r0, r3, 0
  sub r0, r0, 1
  st r0, r3, 0
finalyze_rot:
  move r2, r1
  move r3, tout_rot_cnt
  move r0, TOUT_ROT_SET
  st r0, r3, 0
end_rot:
  jump loop_forever

no_change:
tout_reed:
  move r3, tout_reed_cnt
  ld r0, r3, 0
  jumpr tout_reed_end, 1, lt    ;tout_reed_cnt was 0 => do nothing
  sub r0, r0, 1
  jumpr tout_reed_end, 1, ge    ;timeout not expired
tout_reed_exp:
  stage_inc 1
  jumps tout_reed_end, REED_DIV ,lt         ;reed counter not at max value
  stage_rst                                 ;reed counter at max: reset counter
  move r3, evnt
  move r0, REED_EVNT
  st r0, r3, 0
  wake
tout_reed_end:

tout_btn:
  move r3, tout_btn_cnt
  ld r0, r3, 0
  jumpr tout_btn_end, 1, lt    ; tout_btn_cnt was 0 => do nothing
  sub r0, r0, 1                ; count down timeout
  st r0, r3, 0
tout_btn_end:

tout_rot:
  move r3, tout_rot_cnt
  ld r0, r3, 0
  jumpr tout_rot_end, 1, lt    ; tout_rot_cnt was 0 => do nothing
  sub r0, r0, 1                ; count down timeout
  st r0, r3, 0
  jumpr tout_rot_end, 1, ge    ; timeout not expired
  move r3, evnt
  move r0, ROT_EVNT
  st r0, r3, 0
  wake
tout_rot_end:
  jump loop_forever
