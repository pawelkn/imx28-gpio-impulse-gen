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

#define DRV_NAME "imx28-gpio-pwm"

#define HW_TIMROT_TIMCTRL_REG(n)        (0x20 + (n) * 0x40)
#define HW_TIMROT_TIMCTRL_REG_SET(n)    (HW_TIMROT_TIMCTRL_REG(n) + 4)
#define HW_TIMROT_TIMCTRL_REG_CLR(n)    (HW_TIMROT_TIMCTRL_REG(n) + 8)
#define HW_TIMROT_FIXED_COUNT_REG(n)    (0x40 + (n) * 0x40)

#define TIMROT_TIMCTRL_SELECT_32K       (0xb)
#define TIMROT_TIMCTRL_ALWAYS_TICK      (0xf)
#define TIMROT_TIMCTRL_RELOAD           (1 << 6)
#define TIMROT_TIMCTRL_UPDATE           (1 << 7)
#define TIMROT_TIMCTRL_IRQ_EN           (1 << 14)
#define TIMROT_TIMCTRL_IRQ              (1 << 15)

#define TIMROT_N_DEFAULT    2
#define TIMEOUT_DEFAULT     60000

struct imx28_gpio_pwm_platform_data {
    int gpio;
    bool inverted;

    u32 pulse_width;
    u32 pulse_delay;

    u32 timrot_n;
    u32 timrot_irq;
    void __iomem *timrot_base;
};

struct imx28_gpio_pwm_hw {
    const struct imx28_gpio_pwm_platform_data *pdata;
    struct miscdevice miscdev;

    u64 counter;
    bool state;
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

static irqreturn_t imx28_gpio_pwm_irq(int irq, void *dev_id)
{
    struct imx28_gpio_pwm_hw *hw = dev_id;
    const struct imx28_gpio_pwm_platform_data *pdata = hw->pdata;

    if (hw->counter > 0 ) {
        if (hw->state)
            hw->counter--;

       hw->state = ! hw->state;
       gpio_set_value(pdata->gpio, hw->state ^ pdata->inverted);
    }

    /* restart timer */
    writel(hw->state ? pdata->pulse_width : pdata->pulse_delay,
        pdata->timrot_base + HW_TIMROT_FIXED_COUNT_REG(pdata->timrot_n));

    /* acknowledge the interrupt */
    writel(TIMROT_TIMCTRL_IRQ,
        pdata->timrot_base + HW_TIMROT_TIMCTRL_REG_CLR(pdata->timrot_n));

    return IRQ_HANDLED;
}

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

    pdata = kzalloc(sizeof(struct imx28_gpio_pwm_platform_data), GFP_KERNEL);
    if (!pdata)
        return ERR_PTR(-ENOMEM);

    pdata->gpio = of_get_gpio_flags(np, 0, &flags);
    pdata->inverted = flags & OF_GPIO_ACTIVE_LOW;

    err = of_property_read_u32(np, "pulse-width", &pdata->pulse_width);
    if (err)
        pdata->pulse_width = TIMEOUT_DEFAULT;

    err = of_property_read_u32(np, "pulse-delay", &pdata->pulse_delay);
    if (err)
        pdata->pulse_delay = TIMEOUT_DEFAULT;

    err = of_property_read_u32(np, "timrot-number", &pdata->timrot_n);
    if (err)
        pdata->timrot_n = TIMROT_N_DEFAULT;

    /* parse timrot device tree */
    np = of_find_compatible_node(NULL, NULL, "fsl,timrot");
    pdata->timrot_base = of_iomap(np, 0);
    if (!pdata->timrot_base)
        return ERR_PTR(-ENOMEM);

    pdata->timrot_irq = irq_of_parse_and_map(np, pdata->timrot_n);
    if (pdata->timrot_irq < 0)
        return ERR_PTR(-ENOMEM);

    return pdata;
}

static int imx28_gpio_pwm_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    const struct imx28_gpio_pwm_platform_data *pdata = dev_get_platdata(dev);
    struct imx28_gpio_pwm_hw *hw;
    int err;

    /* Read platform data */
    if (!pdata) {
        pdata = imx28_gpio_pwm_parse_dt(dev);
        if (IS_ERR(pdata))
            return PTR_ERR(pdata);

        if (!pdata) {
            dev_err(dev, "missing or invalid platform data\n");
            return -EINVAL;
        }
    }

    /* Create device data struct */
    hw = kzalloc(sizeof(struct imx28_gpio_pwm_hw), GFP_KERNEL);
    if (!hw) {
        err = -ENOMEM;
        goto exit_free_mem;
    }

    hw->pdata = pdata;
    hw->counter = 0;
    hw->state = 0;

    dev_set_drvdata(dev, hw);

    /* Setup gpio */
    err = gpio_request_one(pdata->gpio, GPIOF_IN, dev_name(dev));
    if (err) {
        dev_err(dev, "unable to request GPIO %d\n", pdata->gpio);
        goto exit_free_mem;
    }

    gpio_direction_output(pdata->gpio, 1);
    gpio_set_value(pdata->gpio, hw->state ^ pdata->inverted);

    /* Setup timer */
    writel(TIMROT_TIMCTRL_UPDATE | TIMROT_TIMCTRL_ALWAYS_TICK |
        TIMROT_TIMCTRL_IRQ_EN,
        pdata->timrot_base + HW_TIMROT_TIMCTRL_REG(pdata->timrot_n));

    /* Setup timer irq */
    err = request_irq(pdata->timrot_irq, &imx28_gpio_pwm_irq, 0, DRV_NAME, hw);
    if (err) {
        dev_err(dev, "failed to register timrot_irq\n");
        goto exit_free_gpio;
    }

    /* Run timer */
    writel(TIMEOUT_DEFAULT,
        pdata->timrot_base + HW_TIMROT_FIXED_COUNT_REG(pdata->timrot_n));

    /* Setup misc devcie */
    hw->miscdev.minor  = MISC_DYNAMIC_MINOR;
    hw->miscdev.name   = dev_name(dev);
    hw->miscdev.fops   = &imx28_gpio_pwm_fops;
    hw->miscdev.parent = dev;

    err = misc_register(&hw->miscdev);
    if (err) {
        dev_err(dev, "failed to register misc device\n");
        goto exit_free_irq;
    }

    dev_info(dev, "registered new misc device %s\n", hw->miscdev.name);
    return 0;

exit_free_irq:
    free_irq(pdata->timrot_irq, hw);
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
    free_irq(pdata->timrot_irq, hw);
    gpio_free(pdata->gpio);
    kfree(hw);
    if (!dev_get_platdata(&pdev->dev))
        kfree(pdata);

    return 0;
}

static int imx28_gpio_pwm_suspend(struct device *dev)
{
    struct imx28_gpio_pwm_hw *hw = dev_get_drvdata(dev);
    const struct imx28_gpio_pwm_platform_data *pdata = hw->pdata;

    if (device_may_wakeup(dev))
        disable_irq_wake(pdata->timrot_irq);

    return 0;
}

static int imx28_gpio_pwm_resume(struct device *dev)
{
    struct imx28_gpio_pwm_hw *hw = dev_get_drvdata(dev);
    const struct imx28_gpio_pwm_platform_data *pdata = hw->pdata;

    if (device_may_wakeup(dev))
        enable_irq_wake(pdata->timrot_irq);

    return 0;
}

static SIMPLE_DEV_PM_OPS(imx28_gpio_pwm_pm_ops,
    imx28_gpio_pwm_suspend, imx28_gpio_pwm_resume);

static struct platform_driver imx28_gpio_pwm_driver = {
    .probe      = imx28_gpio_pwm_probe,
    .remove     = imx28_gpio_pwm_remove,
    .driver     = {
        .name           = DRV_NAME,
        .pm	            = &imx28_gpio_pwm_pm_ops,
        .of_match_table = of_match_ptr(imx28_gpio_pwm_of_match),
    }
};
module_platform_driver(imx28_gpio_pwm_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("i.MX28 GPIO pulse width modulator");
MODULE_AUTHOR("Paweł Knioła <pawel.kn@gmail.com>");
