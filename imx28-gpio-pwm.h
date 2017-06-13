#ifndef IMX28_GPIO_PWM_FIQ_H
#define IMX28_GPIO_PWM_FIQ_H

#define BIT_SET   0x4
#define BIT_CLR	0x8
#define BIT_TOG	0xc

#define HW_PINCTRL_DOUT_REG(n)         (0x700 + (n) * 0x10)

#define HW_ICOLL_INTERRUPTn_REG(n)		(0x120 + (n) * 0x10)

#define HW_TIMROT_TIMCTRL_REG(n)       (0x20 + (n) * 0x40)
#define HW_TIMROT_FIXED_COUNT_REG(n)   (0x40 + (n) * 0x40)

#define TIMROT_TIMCTRL_SELECT_32K      (0xb)
#define TIMROT_TIMCTRL_ALWAYS_TICK     (0xf)
#define TIMROT_TIMCTRL_RELOAD          (1 << 6)
#define TIMROT_TIMCTRL_UPDATE          (1 << 7)
#define TIMROT_TIMCTRL_IRQ_EN          (1 << 14)
#define TIMROT_TIMCTRL_IRQ             (1 << 15)

#define TIMROT_DEFAULT                 2
#define TIMROT_FIXED_COUNT_DEFAULT     60000

#ifdef __ASSEMBLY__
#define __REG_NR(x)     r##x
#else
#define __REG_NR(x)     (x)
#endif

#define reg_timctrl     __REG_NR(8)
#define reg_pinctrl     __REG_NR(9)
#define reg_gpiopin     __REG_NR(10)
#define reg_counter     __REG_NR(11)
#define reg_temp        __REG_NR(12)

#endif
