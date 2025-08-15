/*
 * Copyright (c) 2025, Cody Gu <gujiaqi@iscas.ac.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_ENCODER_STM32_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_ENCODER_STM32_H_

#include <zephyr/drivers/sensor.h>

enum sensor_channel_encoder_stm32 {
	/* Rotate direction */
	SENSOR_CHAN_DIRECTION = SENSOR_ATTR_PRIV_START,
	/* Rotate revolutions all */
	SENSOR_CHAN_REVOLUTIONS,
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_ENCODER_STM32_H_ */
