/*
 * Copyright (c) 2025, Cody	Gu <gujiaqi@iscas.ac.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_
#define ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_

#include <zephyr/device.h>
#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif

enum motor_direction {
    MOTOR_STOP,
    MOTOR_FORWARD,
    MOTOR_REVERSE,
    MOTOR_BRAKE
};

enum motor_channel {
    MOTOR_CHANNEL_A,
    MOTOR_CHANNEL_B
};

typedef int (*motor_driver_set_motor_t)(const struct device *dev,
                                        enum motor_channel channel,
                                        enum motor_direction dir,
                                        uint16_t speed);

__subsystem struct motor_driver_api {
    motor_driver_set_motor_t set_motor;
};

static inline int set_motor(const struct device *dev,
                            enum motor_channel channel,
                            enum motor_direction dir,
                            uint16_t speed) {
	const struct motor_driver_api *api = (const struct motor_driver_api *) dev->api;

	return api->set_motor(dev, channel, dir, speed);
}

#ifdef __cplusplus
}
#endif

#endif // ZEPHYR_INCLUDE_DRIVERS_MOTOR_H_