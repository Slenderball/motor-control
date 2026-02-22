#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/slab.h>

struct motor_data {
    struct gpio_desc *front;
    struct gpio_desc *back;
    struct gpio_desc *move;
    char last_command;
    int current_speed;   
    struct hrtimer pwm_timer;
    bool pwm_pin_high; 
};

#define PWM_PERIOD_NS 10000000 

static enum hrtimer_restart pwm_timer_callback(struct hrtimer *timer)
{
    struct motor_data *data = container_of(timer, struct motor_data, pwm_timer);
    ktime_t now;
    u64 on_time_ns, off_time_ns;

    now = hrtimer_cb_get_time(timer);

    if (data->current_speed == 0) {
        gpiod_set_value(data->move, 0);
        return HRTIMER_NORESTART;
    }
    if (data->current_speed >= 100) {
        gpiod_set_value(data->move, 1);
        return HRTIMER_NORESTART;
    }

    on_time_ns = (PWM_PERIOD_NS * data->current_speed) / 100;
    off_time_ns = PWM_PERIOD_NS - on_time_ns;

    if (data->pwm_pin_high) {
        gpiod_set_value(data->move, 0);
        data->pwm_pin_high = false;
        hrtimer_forward(timer, now, ns_to_ktime(off_time_ns));
    } else {
        gpiod_set_value(data->move, 1);
        data->pwm_pin_high = true;
        hrtimer_forward(timer, now, ns_to_ktime(on_time_ns));
    }

    return HRTIMER_RESTART;
}

static ssize_t motor_set_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct motor_data *data = dev_get_drvdata(dev);
    return sprintf(buf, "Status -> Dir: %c, Speed: %d%%\n", 
                   data->last_command ? data->last_command : 's', 
                   data->current_speed);
}

static ssize_t motor_set_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct motor_data *data = dev_get_drvdata(dev);
    char dir;
    int speed;
    int ret;

    if (count < 4) return -EINVAL; 

    dir = buf[0];
    ret = sscanf(buf + 1, "%3d", &speed);
    if (ret != 1) return -EINVAL;

    if (speed > 100) speed = 100;
    if (speed < 0) speed = 0;    

    hrtimer_cancel(&data->pwm_timer);

    if (dir == 'f') {
        gpiod_set_value(data->front, 1);
        gpiod_set_value(data->back, 0);
    } else if (dir == 'b') {
        gpiod_set_value(data->front, 0);
        gpiod_set_value(data->back, 1);
    } else {
        gpiod_set_value(data->front, 0);
        gpiod_set_value(data->back, 0);
        speed = 0;
        dir = 's';
    }

    data->last_command = dir;
    data->current_speed = speed;

    if (speed > 0 && speed < 100) {
        data->pwm_pin_high = true; 
        gpiod_set_value(data->move, 1);
        u64 first_period = (PWM_PERIOD_NS * speed) / 100;
        hrtimer_start(&data->pwm_timer, ns_to_ktime(first_period), HRTIMER_MODE_REL);
    } else if (speed >= 100) {
        gpiod_set_value(data->move, 1);
    } else {
        gpiod_set_value(data->move, 0);
    }

    return count; 
}

static DEVICE_ATTR_RW(motor_set);

static int motor_probe(struct platform_device *pdev)
{
    struct motor_data *data;
    int ret;

    dev_info(&pdev->dev, "Probe function called!\n");

    data = devm_kzalloc(&pdev->dev, sizeof(struct motor_data), GFP_KERNEL);
    if (!data) return -ENOMEM;

    data->front = devm_gpiod_get(&pdev->dev, "front", GPIOD_OUT_LOW);
    if (IS_ERR(data->front)) return PTR_ERR(data->front);

    data->back = devm_gpiod_get(&pdev->dev, "back", GPIOD_OUT_LOW);
    if (IS_ERR(data->back)) return PTR_ERR(data->back);

    data->move = devm_gpiod_get(&pdev->dev, "move", GPIOD_OUT_LOW);
    if (IS_ERR(data->move)) return PTR_ERR(data->move);

    platform_set_drvdata(pdev, data);

    hrtimer_init(&data->pwm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    data->pwm_timer.function = pwm_timer_callback;

    ret = device_create_file(&pdev->dev, &dev_attr_motor_set);
    if (ret) {
        dev_err(&pdev->dev, "Failed to create sysfs file\n");
        return ret;
    }

    dev_info(&pdev->dev, "All systems GREEN.\n");
    return 0;
}

static void motor_remove(struct platform_device *pdev)
{
    struct motor_data *data = platform_get_drvdata(pdev);

    hrtimer_cancel(&data->pwm_timer);
    
    if (data->front) gpiod_set_value(data->front, 0);
    if (data->back) gpiod_set_value(data->back, 0);
    if (data->move) gpiod_set_value(data->move, 0);

    device_remove_file(&pdev->dev, &dev_attr_motor_set);

    dev_info(&pdev->dev, "Driver removed.\n");
}

static const struct of_device_id motor_dt_ids[] = {
    { .compatible = "slend,super-motor" },
    { }
};
MODULE_DEVICE_TABLE(of, motor_dt_ids);

static struct platform_driver motor_driver = {
    .probe = motor_probe,
    .remove = motor_remove,
        .name = "my_motor_test_driver",
        .of_match_table = motor_dt_ids,
    },
};

module_platform_driver(motor_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Slend");
MODULE_DESCRIPTION("Soft PWM Motor Driver");        