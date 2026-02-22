    #include <linux/module.h>
    #include <linux/platform_device.h>
    #include <linux/gpio/consumer.h>
    #include <linux/of.h>
    #include<linux/sysfs.h>
    #include <linux/interrupt.h>
    #include <linux/ktime.h>
    #include <linux/math64.h>

    const short int holes = 20;
    u64 pulse_count = 0;
    struct speed_data
    {
        u64 last_time;
        u64 rpm;
        int irq;
    };

    struct speed_data sensor_state= {0, 0, 0};

    static irqreturn_t speed_irq_handler(int irq, void *dev_id)
    {
        u64 start;
        u64 delta;

        start = ktime_to_ns(ktime_get());
        if (sensor_state.last_time !=0){
            delta = start - sensor_state.last_time;
            if (delta > 1000){
                sensor_state.rpm = div64_u64(60000000000ULL, delta * holes);
            }
        }
        sensor_state.last_time = start;

        return IRQ_HANDLED;
    }

    static ssize_t show_speed(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
    {
        u64 now = ktime_to_ns(ktime_get());
        u64 time_diff = now - sensor_state.last_time;

        if (time_diff > 500000000ULL) {
            sensor_state.rpm = 0;
        }
        
        return sprintf(buf, "%llu\n", sensor_state.rpm);
    }

    static struct kobj_attribute speed_attr = __ATTR(speed, 0444, show_speed, NULL);

    static int speed_probe(struct platform_device *pdev)
    {
        struct gpio_desc *speed_status;
        int ret;
        int irq;
        sensor_state.last_time = 0;
        sensor_state.rpm = 0;

        printk(KERN_INFO "SPEED_TEST: Probe function called! I matched with Device Tree!\n");

        speed_status = devm_gpiod_get(&pdev->dev, "speed", GPIOD_IN);
        if (IS_ERR(speed_status)) {
            printk(KERN_ERR "SPEED_TEST: Error! Could not find 'speed' GPIO.\n");
            return PTR_ERR(speed_status);
        }
        printk(KERN_INFO "SPEED_TEST: Speed GPIO found successfully.\n");

        irq = gpiod_to_irq(speed_status);
        if (irq < 0) {
            printk(KERN_ERR "SPEED_TEST: Error! Could not get IRQ number for 'speed' GPIO.\n");
            return irq;
        }
        
        printk(KERN_INFO "SPEED_TEST: All systems GREEN. Ready for logic.\n");

        ret = devm_request_irq(&pdev->dev, irq, speed_irq_handler, IRQF_TRIGGER_RISING , "my_speed_irq", NULL);

        ret = sysfs_create_file(kernel_kobj, &speed_attr.attr);

        return 0;
    }

    static void speed_remove(struct platform_device *pdev)
    {
        printk(KERN_INFO "SPEED_TEST: Driver removed. Goodbye!\n");
        sysfs_remove_file(kernel_kobj, &speed_attr.attr);
    }

    static const struct of_device_id speed_dt_ids[] = {
        { .compatible = "slend,super-speed" },
        {}
    };
    MODULE_DEVICE_TABLE(of, speed_dt_ids);

    static struct platform_driver speed_driver = {
        .probe = speed_probe,
        .remove = speed_remove,
        .driver = {
            .name = "my_speed_test_driver",
            .of_match_table = speed_dt_ids,
        },
    };

    module_platform_driver(speed_driver);

    MODULE_LICENSE("GPL");
    MODULE_AUTHOR("Slend");
    MODULE_DESCRIPTION("Minimal Device Tree Test");
