/*
 * (c) 2017 Paweł Knioła <pawel.kn@gmail.com>
 *
 * i.MX28 GPIO pulse width modulator
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pm.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/time.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/fiq.h>

#include <linux/platform_data/asoc-imx-ssi.h>

#include "imx28-gpio-pwm.h"

#define DRV_NAME "imx28-gpio-pwm"

struct imx28_gpio_pwm_platform_data {
    int gpio;

    u32 timrot;
    u32 timrot_irq;
    u32 timrot_fixed_count;

    void __iomem *timrot_base;
    void __iomem *pinctrl_base;
    void __iomem *icoll_base;
};

struct imx28_gpio_pwm_hw {
    const struct imx28_gpio_pwm_platform_data *pdata;
    struct miscdevice miscdev;

    u64 counter;
    bool state;
};

extern unsigned char imx28_gpio_pwm_fiq_handler, imx28_gpio_pwm_fiq_handler_end;
static struct fiq_handler imx28_gpio_pwm_fh = {
    .name = DRV_NAME
};

static int imx28_gpio_pwm_open(struct inode *inode, struct file *file)
{
   /* dummy open function must be declared due to bug in kernel 3.16 */
   return 0;
}

static ssize_t imx28_gpio_pwm_read(struct file *file, char __user * userbuf, size_t count, loff_t * ppos)
{
    struct miscdevice *miscdev = file->private_data;
    struct device *dev = miscdev->parent;
    struct imx28_gpio_pwm_hw *hw = dev_get_drvdata(dev);

    char buf[22];
    int len = sprintf(buf, "%llu\n", hw->counter);

    if ((len < 0) || (len > count))
        return -EINVAL;

    if (*ppos != 0)
        return 0;

    if (copy_to_user(userbuf, buf, len))
        return -EINVAL;

    *ppos = len;
    return len;
}

static ssize_t imx28_gpio_pwm_write(struct file *file, const char __user * userbuf, size_t count, loff_t * ppos)
{
    struct miscdevice *miscdev = file->private_data;
    struct device *dev = miscdev->parent;
    struct imx28_gpio_pwm_hw *hw = dev_get_drvdata(dev);

    u64 increment;
    if (kstrtoull_from_user(userbuf, count, 0, &increment))
        return -EINVAL;

    hw->counter += increment;
    return count;
}

static struct file_operations imx28_gpio_pwm_fops = {
    .owner  = THIS_MODULE,
    .open   = imx28_gpio_pwm_open,
    .read   = imx28_gpio_pwm_read,
    .write  = imx28_gpio_pwm_write,
};

static const struct of_device_id imx28_gpio_pwm_of_match[] = {
    { .compatible = DRV_NAME, },
    { },
};
MODULE_DEVICE_TABLE(of, imx28_gpio_pwm_of_match);

static struct imx28_gpio_pwm_platform_data *imx28_gpio_pwm_parse_dt(struct device *dev)
{
    enum of_gpio_flags flags;
    struct imx28_gpio_pwm_platform_data *pdata;
    const struct of_device_id *of_id;
    struct device_node *np;
    int err;

    /* parse driver device tree */
    of_id = of_match_device(imx28_gpio_pwm_of_match, dev);
    np = dev->of_node;

    if (!of_id || !np)
        return NULL;

    pdata = devm_kzalloc(dev, sizeof(struct imx28_gpio_pwm_platform_data), GFP_KERNEL);
    if (!pdata)
        return ERR_PTR(-ENOMEM);

    pdata->gpio = of_get_gpio_flags(np, 0, &flags);

    err = of_property_read_u32(np, "timrot", &pdata->timrot);
    if (err)
        pdata->timrot = TIMROT_DEFAULT;

    err = of_property_read_u32(np, "timrot-fixed-count", &pdata->timrot_fixed_count);
    if (err)
        pdata->timrot_fixed_count = TIMROT_FIXED_COUNT_DEFAULT;

    /* parse timrot device tree */
    np = of_find_compatible_node(NULL, NULL, "fsl,imx28-timrot");
    pdata->timrot_base = of_iomap(np, 0);
    if (!pdata->timrot_base)
        return ERR_PTR(-ENOMEM);

    pdata->timrot_irq = irq_of_parse_and_map(np, pdata->timrot);
    if (pdata->timrot_irq < 0)
        return ERR_PTR(-ENOMEM);

    /* parse pin control device tree */
    np = of_find_compatible_node(NULL, NULL, "fsl,imx28-pinctrl");
    pdata->pinctrl_base = of_iomap(np, 0);
    if (!pdata->pinctrl_base)
        return ERR_PTR(-ENOMEM);

    /* parse interrupt controller device tree */
    np = of_find_compatible_node(NULL, NULL, "fsl,imx28-icoll");
    pdata->icoll_base = of_iomap(np, 0);
    if (!pdata->icoll_base)
        return ERR_PTR(-ENOMEM);

    return pdata;
}

static int imx28_gpio_pwm_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    const struct imx28_gpio_pwm_platform_data *pdata = dev_get_platdata(dev);
    struct imx28_gpio_pwm_hw *hw;
    struct irq_data* irqd;
    struct pt_regs regs;
    long gpio_pin, gpio_bank;
    int err;

    /* read platform data */
    if (!pdata) {
        pdata = imx28_gpio_pwm_parse_dt(dev);
        if (IS_ERR(pdata))
            return PTR_ERR(pdata);

        if (!pdata) {
            dev_err(dev, "missing or invalid platform data\n");
            return -EINVAL;
        }
    }

    /* create device data struct */
    hw = devm_kzalloc(dev, sizeof(struct imx28_gpio_pwm_hw), GFP_KERNEL);
    if (!hw) {
        err = -ENOMEM;
        goto exit_free_mem;
    }

    hw->pdata = pdata;
    hw->counter = 0;
    hw->state = 0;

    dev_set_drvdata(dev, hw);

    /* setup gpio */
    err = gpio_request_one(pdata->gpio, GPIOF_IN, dev_name(dev));
    if (err) {
        dev_err(dev, "unable to request GPIO %d\n", pdata->gpio);
        goto exit_free_mem;
    }

    gpio_bank = pdata->gpio / 32;
    gpio_pin = (1 << (pdata->gpio % 32));

    gpio_direction_output(pdata->gpio, 1);
    gpio_set_value(pdata->gpio, 0);

    /* setup timer */
    writel(TIMROT_TIMCTRL_UPDATE | TIMROT_TIMCTRL_ALWAYS_TICK |
        TIMROT_TIMCTRL_RELOAD | TIMROT_TIMCTRL_IRQ_EN,
        pdata->timrot_base + HW_TIMROT_TIMCTRL_REG(pdata->timrot));

    /* setup FIQ */
    err = claim_fiq(&imx28_gpio_pwm_fh);
    if (err) {
        dev_err(dev, "failed to register timrot FIQ\n");
        goto exit_free_gpio;
    }

    set_fiq_handler(&imx28_gpio_pwm_fiq_handler,
        &imx28_gpio_pwm_fiq_handler_end - &imx28_gpio_pwm_fiq_handler);

    regs.uregs[reg_timctrl] = (long)pdata->timrot_base + HW_TIMROT_TIMCTRL_REG(pdata->timrot);
    regs.uregs[reg_pinctrl] = (long)pdata->pinctrl_base + HW_PINCTRL_DOUT_REG(gpio_bank);
    regs.uregs[reg_gpiopin] = (long)gpio_pin;
    regs.uregs[reg_counter] = (long)&hw->counter;
    set_fiq_regs(&regs);

    /* change the timer's IRQ to FIR*/
    irqd = irq_get_irq_data(pdata->timrot_irq);
    writel(1 << 4,
       pdata->icoll_base + HW_ICOLL_INTERRUPTn_REG(irqd->hwirq) + BIT_SET);

    /* enable the FIQ */
    enable_fiq(pdata->timrot_irq);

    /* run timer */
    writel(pdata->timrot_fixed_count,
        pdata->timrot_base + HW_TIMROT_FIXED_COUNT_REG(pdata->timrot));

    /* setup misc devcie */
    hw->miscdev.minor  = MISC_DYNAMIC_MINOR;
    hw->miscdev.name   = dev_name(dev);
    hw->miscdev.fops   = &imx28_gpio_pwm_fops;
    hw->miscdev.parent = dev;

    err = misc_register(&hw->miscdev);
    if (err) {
        dev_err(dev, "failed to register misc device\n");
        goto exit_free_gpio;
    }

    dev_info(dev, "registered new misc device %s\n", hw->miscdev.name);
    return 0;

exit_free_gpio:
    gpio_free(pdata->gpio);
exit_free_mem:
    kfree(hw);
    if (!dev_get_platdata(&pdev->dev))
        kfree(pdata);

    return err;
}

static int imx28_gpio_pwm_remove(struct platform_device *pdev)
{
    struct imx28_gpio_pwm_hw *hw = platform_get_drvdata(pdev);
    const struct imx28_gpio_pwm_platform_data *pdata = hw->pdata;

    device_init_wakeup(&pdev->dev, false);

    misc_deregister(&hw->miscdev);
    gpio_free(pdata->gpio);
    kfree(hw);
    if (!dev_get_platdata(&pdev->dev))
        kfree(pdata);

    return 0;
}

static struct platform_driver imx28_gpio_pwm_driver = {
    .probe      = imx28_gpio_pwm_probe,
    .remove     = imx28_gpio_pwm_remove,
    .driver     = {
        .name           = DRV_NAME,
        .of_match_table = of_match_ptr(imx28_gpio_pwm_of_match),
    }
};
module_platform_driver(imx28_gpio_pwm_driver);

MODULE_LICENSE("GPL 2");
MODULE_DESCRIPTION("i.MX28 GPIO pulse width modulator");
MODULE_AUTHOR("Paweł Knioła <pawel.kn@gmail.com>");
