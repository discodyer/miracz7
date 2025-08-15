/*
 * Copyright (c) 2025, Cody	Gu <gujiaqi@iscas.ac.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT toshiba_tb6612fng

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/motor.h>

LOG_MODULE_REGISTER(tb6612, CONFIG_MOTOR_DRIVER_LOG_LEVEL);

struct tb6612_dev_cfg {
    struct pwm_dt_spec pwm_a;
    struct pwm_dt_spec pwm_b;
    struct gpio_dt_spec ain1;
    struct gpio_dt_spec ain2;
    struct gpio_dt_spec bin1;
    struct gpio_dt_spec bin2;
    struct gpio_dt_spec stby;
    uint32_t pwm_period;
};

struct tb6612_dev_data {
    bool initialized;
};

static int tb6612_init(const struct device *dev)
{
    const struct tb6612_dev_cfg *dev_cfg = dev->config;
    struct tb6612_dev_data *dev_data = dev->data;
    int ret;

    if (!device_is_ready(dev_cfg->pwm_a.dev)) {
        LOG_ERR("PWM device A not ready");
        return -ENODEV;
    }

    if (!device_is_ready(dev_cfg->pwm_b.dev)) {
        LOG_ERR("PWM device B not ready");
        return -ENODEV;
    }

    /* 初始化GPIO */
    if (!gpio_is_ready_dt(&dev_cfg->ain1) ||
        !gpio_is_ready_dt(&dev_cfg->ain2) ||
        !gpio_is_ready_dt(&dev_cfg->bin1) ||
        !gpio_is_ready_dt(&dev_cfg->bin2) ||
        !gpio_is_ready_dt(&dev_cfg->stby)) {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&dev_cfg->ain1, GPIO_OUTPUT_INACTIVE);
    ret |= gpio_pin_configure_dt(&dev_cfg->ain2, GPIO_OUTPUT_INACTIVE);
    ret |= gpio_pin_configure_dt(&dev_cfg->bin1, GPIO_OUTPUT_INACTIVE);
    ret |= gpio_pin_configure_dt(&dev_cfg->bin2, GPIO_OUTPUT_INACTIVE);
    ret |= gpio_pin_configure_dt(&dev_cfg->stby, GPIO_OUTPUT_ACTIVE);
    
    if (ret != 0) {
        LOG_ERR("Failed to configure GPIO pins");
        return ret;
    }

    dev_data->initialized = true;
    LOG_INF("TB6612 initialized");
    return 0;
}

/**
 * @brief Set motor state
 * 
 * @param dev Motor device instance
 * @param ch Channel (A or B)
 * @param dir Direction (STOP/FORWARD/REVERSE/BRAKE)
 * @param speed Speed (0-100%)
 * @return int 0 on success, negative errno on error
 */
static int tb6612_set_motor(const struct device *dev,
                           enum motor_channel channel,
                           enum motor_direction dir,
                           uint16_t speed)
{
    const struct tb6612_dev_cfg *dev_cfg = dev->config;
    int ret;

    if (!device_is_ready(dev)) {
        return -ENODEV;
    }

    /* 确保速度在0-100%范围内 */
    speed = CLAMP(speed, 0, 100);

    switch (channel) {
    case MOTOR_CHANNEL_A: /* 通道A */
        switch (dir) {
        case MOTOR_FORWARD:
            gpio_pin_set_dt(&dev_cfg->ain1, 1);
            gpio_pin_set_dt(&dev_cfg->ain2, 0);
            break;
        case MOTOR_REVERSE:
            gpio_pin_set_dt(&dev_cfg->ain1, 0);
            gpio_pin_set_dt(&dev_cfg->ain2, 1);
            break;
        case MOTOR_STOP:
            gpio_pin_set_dt(&dev_cfg->ain1, 0);
            gpio_pin_set_dt(&dev_cfg->ain2, 0);
            break;
        case MOTOR_BRAKE:
            gpio_pin_set_dt(&dev_cfg->ain1, 1);
            gpio_pin_set_dt(&dev_cfg->ain2, 1);
            break;
        }

        ret = pwm_set_pulse_dt(&dev_cfg->pwm_a, (dev_cfg->pwm_a.period * speed) / 100);
        if (ret < 0) {
            LOG_ERR("Failed to set PWM A (err %d)", ret);
            return ret;
        }
        break;

    case MOTOR_CHANNEL_B: /* 通道B */
        switch (dir) {
        case MOTOR_FORWARD:
            gpio_pin_set_dt(&dev_cfg->bin1, 1);
            gpio_pin_set_dt(&dev_cfg->bin2, 0);
            break;
        case MOTOR_REVERSE:
            gpio_pin_set_dt(&dev_cfg->bin1, 0);
            gpio_pin_set_dt(&dev_cfg->bin2, 1);
            break;
        case MOTOR_STOP:
            gpio_pin_set_dt(&dev_cfg->bin1, 0);
            gpio_pin_set_dt(&dev_cfg->bin2, 0);
            break;
        case MOTOR_BRAKE:
            gpio_pin_set_dt(&dev_cfg->bin1, 1);
            gpio_pin_set_dt(&dev_cfg->bin2, 1);
            break;
        }

        ret = pwm_set_pulse_dt(&dev_cfg->pwm_b, (dev_cfg->pwm_b.period * speed) / 100);
        if (ret < 0) {
            LOG_ERR("Failed to set PWM B (err %d)", ret);
            return ret;
        }
        break;

    default:
        return -EINVAL;
    }

    return 0;
}

static const struct motor_driver_api tb6612_motor_api = {
    .set_motor = tb6612_set_motor,
};

#define TB6612_INIT(n) \
    static struct tb6612_dev_data tb6612_dev_data_##n; \
    \
    static const struct tb6612_dev_cfg tb6612_dev_cfg_##n = { \
        .pwm_a = PWM_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), 0), \
        .pwm_b = PWM_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), 1), \
        .ain1 = GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), ain1_gpios, 0), \
        .ain2 = GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), ain2_gpios, 0), \
        .bin1 = GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), bin1_gpios, 0), \
        .bin2 = GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), bin2_gpios, 0), \
        .stby = GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), stby_gpios, 0), \
    }; \
    \
    DEVICE_DT_INST_DEFINE(n, \
                         tb6612_init, \
                         NULL, \
                         &tb6612_dev_data_##n, \
                         &tb6612_dev_cfg_##n, \
                         POST_KERNEL, \
                         CONFIG_MOTOR_DRIVER_INIT_PRIORITY, \
                         &tb6612_motor_api);

DT_INST_FOREACH_STATUS_OKAY(TB6612_INIT)