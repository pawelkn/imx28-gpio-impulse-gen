/*
 * gpio_pulse_generator.c
 *
 * (c) 2017 Paweł Knioła <pawel.kn@gmail.com>
 *
 * Generic GPIO impulse generator.
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
#include <linux/pm.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/time.h>

#include <asm/uaccess.h>

#define DRV_NAME "gpio-pulse-generator"

struct gpio_pulse_generator_platform_data {
    int gpio;
    bool inverted;
    u32 width_ms;
    u32 delay_ms;
};

struct gpio_pulse_generator {
    const struct gpio_pulse_generator_platform_data *pdata;
    struct miscdevice miscdev;
    struct delayed_work work;

    int irq;
    u64 count;
    s64 last_ns;
    bool last_state;
    bool last_stable_state;
};

static int gpio_pulse_generator_open(struct inode *inode, struct file *file)
{
   // dummy open function must be declared due to bug in kernel 3.16
   return 0;
}

static ssize_t gpio_pulse_generator_read(struct file *file, char __user * userbuf, size_t count, loff_t * ppos)
{
    struct miscdevice *miscdev = file->private_data;
    struct device *dev = miscdev->parent;
    struct gpio_pulse_generator *generator = dev_get_drvdata(dev);

    char buf[22];
    int len = sprintf(buf, "%llu\n", generator->count);

    if ((len < 0) || (len > count))
        return -EINVAL;

    if (*ppos != 0)
        return 0;

    if (copy_to_user(userbuf, buf, len))
        return -EINVAL;

    *ppos = len;
    return len;
}

static ssize_t gpio_pulse_generator_write(struct file *file, const char __user * userbuf, size_t count, loff_t * ppos)
{
    struct miscdevice *miscdev = file->private_data;
    struct device *dev = miscdev->parent;
    struct gpio_pulse_generator *generator = dev_get_drvdata(dev);

    u64 increment;
    if (kstrtoull_from_user(userbuf, count, 0, &generator->count))
        return -EINVAL;

    //generator->count += increment;
    return count;
}

static struct file_operations gpio_pulse_generator_fops = {
    .owner = THIS_MODULE,
    .open = gpio_pulse_generator_open,
    .read = gpio_pulse_generator_read,
    .write = gpio_pulse_generator_write,
};

static bool gpio_pulse_generator_get_state(const struct gpio_pulse_generator_platform_data *pdata)
{
    bool state = gpio_get_value(pdata->gpio);
    if (pdata->inverted)
        state = !state;

    return state;
}

static s64 gpio_pulse_generator_get_time_nsec(void)
{
    struct timespec ts;
    getnstimeofday (&ts);
    return timespec_to_ns (&ts);
}

static void gpio_pulse_generator_process_state_change(struct gpio_pulse_generator *generator)
{
    bool state;
    s64 current_ns;
    s64 delta_ns = 0;
    s64 debounce_ns;

    state = gpio_pulse_generator_get_state (generator->pdata);
    current_ns = gpio_pulse_generator_get_time_nsec();
    delta_ns = current_ns - generator->last_ns;
    debounce_ns = (s64)generator->pdata->delay_ms * NSEC_PER_USEC;

    if (delta_ns > debounce_ns) {
        if (generator->last_state && !generator->last_stable_state)
            generator->count++;

        generator->last_stable_state = generator->last_state;
    }

    generator->last_state = state;
    generator->last_ns = current_ns;
}

static void gpio_pulse_generator_delayed_work(struct work_struct *work)
{
    struct gpio_pulse_generator *generator = container_of(work, struct gpio_pulse_generator, work.work);
    gpio_pulse_generator_process_state_change (generator);
}

static irqreturn_t gpio_pulse_generator_irq(int irq, void *dev_id)
{
    struct gpio_pulse_generator *generator = dev_id;
    gpio_pulse_generator_process_state_change (generator);

    if (delayed_work_pending (&generator->work))
        cancel_delayed_work (&generator->work);

    schedule_delayed_work (&generator->work,
        usecs_to_jiffies (generator->pdata->delay_ms));

    return IRQ_HANDLED;
}

static const struct of_device_id gpio_pulse_generator_of_match[] = {
    { .compatible = "gpio-pulse-generator", },
    { },
};
MODULE_DEVICE_TABLE(of, gpio_pulse_generator_of_match);

static struct gpio_pulse_generator_platform_data *gpio_pulse_generator_parse_dt(struct device *dev)
{
    const struct of_device_id *of_id =
                of_match_device(gpio_pulse_generator_of_match, dev);
    struct device_node *np = dev->of_node;
    struct gpio_pulse_generator_platform_data *pdata;
    enum of_gpio_flags flags;
    int err;

    if (!of_id || !np)
        return NULL;

    pdata = kzalloc(sizeof(struct gpio_pulse_generator_platform_data), GFP_KERNEL);
    if (!pdata)
        return ERR_PTR(-ENOMEM);

    pdata->gpio = of_get_gpio_flags(np, 0, &flags);
    pdata->inverted = flags & OF_GPIO_ACTIVE_LOW;
    
    err = of_property_read_u32(np, "width-ms", &pdata->width_ms);
    if (err)
        pdata->width_ms = 5;


    err = of_property_read_u32(np, "delay-ms", &pdata->delay_ms);
    if (err)
        pdata->delay_ms = 5;

    return pdata;
}

static int gpio_pulse_generator_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    const struct gpio_pulse_generator_platform_data *pdata = dev_get_platdata(dev);
    struct gpio_pulse_generator *generator;
    int err;

    if (!pdata) {
        pdata = gpio_pulse_generator_parse_dt(dev);
        if (IS_ERR(pdata))
            return PTR_ERR(pdata);

        if (!pdata) {
            dev_err(dev, "missing platform data\n");
            return -EINVAL;
        }
    }

    generator = kzalloc(sizeof(struct gpio_pulse_generator), GFP_KERNEL);
    if (!generator) {
        err = -ENOMEM;
        goto exit_free_mem;
    }

    generator->pdata = pdata;
    generator->count = 0;

    err = gpio_request_one(pdata->gpio, GPIOF_IN, dev_name(dev));
    if (err) {
        dev_err(dev, "unable to request GPIO %d\n", pdata->gpio);
        goto exit_free_mem;
    }

    generator->last_stable_state = gpio_pulse_generator_get_state (pdata);
    generator->last_state = generator->last_stable_state;
    generator->irq = gpio_to_irq (pdata->gpio);
    generator->last_ns = gpio_pulse_generator_get_time_nsec();

    err = request_irq(generator->irq, &gpio_pulse_generator_irq,
                      IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                      DRV_NAME, generator);
    if (err) {
        dev_err(dev, "unable to request IRQ %d\n", generator->irq);
        goto exit_free_gpio;
    }

    dev_set_drvdata(dev, generator);

    generator->miscdev.minor  = MISC_DYNAMIC_MINOR;
    generator->miscdev.name   = dev_name(dev);
    generator->miscdev.fops   = &gpio_pulse_generator_fops;
    generator->miscdev.parent = dev;

    INIT_DELAYED_WORK(&generator->work, gpio_pulse_generator_delayed_work);

    err = misc_register(&generator->miscdev);
    if (err) {
        dev_err(dev, "failed to register misc device\n");
        goto exit_free_irq;
    }

    dev_info(dev, "registered new misc device %s\n", generator->miscdev);
    return 0;

exit_free_irq:
    free_irq(generator->irq, generator);
exit_free_gpio:
    gpio_free(pdata->gpio);
exit_free_mem:
    kfree(generator);
    if (!dev_get_platdata(&pdev->dev))
        kfree(pdata);

    return err;
}

static int gpio_pulse_generator_remove(struct platform_device *pdev)
{
    struct gpio_pulse_generator *generator = platform_get_drvdata(pdev);
    const struct gpio_pulse_generator_platform_data *pdata = generator->pdata;

    device_init_wakeup(&pdev->dev, false);

    if (delayed_work_pending(&generator->work))
        cancel_delayed_work(&generator->work);

    misc_deregister(&generator->miscdev);
    free_irq(generator->irq, generator);
    gpio_free(pdata->gpio);
    kfree(generator);
    if (!dev_get_platdata(&pdev->dev))
        kfree(pdata);

    return 0;
}

static int gpio_pulse_generator_suspend(struct device *dev)
{
    struct gpio_pulse_generator *generator = dev_get_drvdata(dev);

    if (device_may_wakeup(dev))
        disable_irq_wake(generator->irq);

    return 0;
}

static int gpio_pulse_generator_resume(struct device *dev)
{
    struct gpio_pulse_generator *generator = dev_get_drvdata(dev);

    if (device_may_wakeup(dev))
        enable_irq_wake(generator->irq);

    return 0;
}

static SIMPLE_DEV_PM_OPS(gpio_pulse_generator_pm_ops,
         gpio_pulse_generator_suspend, gpio_pulse_generator_resume);

static struct platform_driver gpio_pulse_generator_driver = {
    .probe		= gpio_pulse_generator_probe,
    .remove		= gpio_pulse_generator_remove,
    .driver		= {
        .name	= DRV_NAME,
        .pm	= &gpio_pulse_generator_pm_ops,
        .of_match_table = of_match_ptr(gpio_pulse_generator_of_match),
    }
};
module_platform_driver(gpio_pulse_generator_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Generic GPIO pulse generator driver");
MODULE_AUTHOR("Paweł Knioła <pawel.kn@gmail.com>");
