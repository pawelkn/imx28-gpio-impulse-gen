/*
 * Copyright (C) 2017 Paweł Knioła
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/linkage.h>
#include <asm/assembler.h>

#include "imx28-gpio-pwm.h"

   .text
   .global imx28_gpio_pwm_fiq_handler_end

ENTRY(imx28_gpio_pwm_fiq_handler)

   /* acknowledge the interrupt */
   mov   reg_temp, #TIMROT_TIMCTRL_IRQ
   str   reg_temp, [reg_timctrl, #BIT_CLR]

   /* return, if counter is eqal 0 */
   ldr   reg_temp, [reg_counter]
   cmp   reg_temp, #0
   beq   return

   /* swap a current gpio state */
   ldr   reg_temp, [reg_pinctrl]
   and   reg_temp, reg_temp, reg_gpiopin
   cmp   reg_temp, #0
   beq   set

   /* decrement counter before the clear slope */
   ldr   reg_temp, [reg_counter]
   sub   reg_temp, #1
   str   reg_temp, [reg_counter]

clear:
   str   reg_gpiopin, [reg_pinctrl, #BIT_CLR]
   b     return

set:
   str   reg_gpiopin, [reg_pinctrl, #BIT_SET]

return:
   /* return from FIQ */
   subs   pc, lr, #4

imx28_gpio_pwm_fiq_handler_end:
