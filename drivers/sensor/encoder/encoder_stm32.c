/*
 * Copyright (c) 2022, Valerio Setti <vsetti@baylibre.com>
 * Copyright (c) 2025, Cody	Gu <gujiaqi@iscas.ac.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_encoder

/** @file
 * @brief STM32 family Quadrature Decoder (QDEC) driver.
 */

#include <errno.h>

#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/logging/log.h>

#include <stm32_ll_tim.h>

LOG_MODULE_REGISTER(encoder_stm32, CONFIG_SENSOR_LOG_LEVEL);

/* Device constant configuration parameters */
struct encoder_stm32_dev_cfg {
	const struct pinctrl_dev_config *pin_config;
	struct stm32_pclken pclken;
	TIM_TypeDef *timer_inst;
	uint32_t encoder_mode;
	bool is_input_polarity_inverted;
	uint8_t input_filtering_level;
	uint32_t counts_per_revolution;
};

/* Device run time data */
struct encoder_stm32_dev_data {
	uint32_t position;
};

static int encoder_stm32_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct encoder_stm32_dev_data *dev_data = dev->data;
	const struct encoder_stm32_dev_cfg *dev_cfg = dev->config;
	uint32_t counter_value;

	if ((chan != SENSOR_CHAN_ALL) && (chan != SENSOR_CHAN_ROTATION)) {
		return -ENOTSUP;
	}

	/* We're only interested in the remainder between the current counter value and
	 * counts_per_revolution. The integer part represents an entire rotation so it
	 * can be ignored
	 */
	counter_value = LL_TIM_GetCounter(dev_cfg->timer_inst) % dev_cfg->counts_per_revolution;

	/* The angle calculated in the fixed-point format (Q26.6 format) */
	dev_data->position = (counter_value * 23040) / dev_cfg->counts_per_revolution;

	printk("USING OUT OF TREE DRIVER\n");

	return 0;
}

static int encoder_stm32_get(const struct device *dev, enum sensor_channel chan,
			  struct sensor_value *val)
{
	struct encoder_stm32_dev_data *const dev_data = dev->data;

	if (chan == SENSOR_CHAN_ROTATION) {
		val->val1 = dev_data->position >> 6;
		val->val2 = (dev_data->position & 0x3F) * 15625;
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int encoder_stm32_initialize(const struct device *dev)
{
	const struct encoder_stm32_dev_cfg *const dev_cfg = dev->config;
	int retval;
	LL_TIM_ENCODER_InitTypeDef init_props;
	uint32_t max_counter_value;

	retval = pinctrl_apply_state(dev_cfg->pin_config, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		return retval;
	}

	if (!device_is_ready(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE))) {
		LOG_ERR("Clock control device not ready");
		return -ENODEV;
	}

	retval = clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
				  (clock_control_subsys_t)&dev_cfg->pclken);
	if (retval < 0) {
		LOG_ERR("Could not initialize clock");
		return retval;
	}

	if (dev_cfg->counts_per_revolution < 1) {
		LOG_ERR("Invalid number of counts per revolution (%d)",
			dev_cfg->counts_per_revolution);
		return -EINVAL;
	}

	LL_TIM_ENCODER_StructInit(&init_props);

	init_props.EncoderMode = dev_cfg->encoder_mode;

	if (dev_cfg->is_input_polarity_inverted) {
		init_props.IC1Polarity = LL_TIM_IC_POLARITY_FALLING;
		init_props.IC2Polarity = LL_TIM_IC_POLARITY_FALLING;
	}

	init_props.IC1Filter = dev_cfg->input_filtering_level * LL_TIM_IC_FILTER_FDIV1_N2;
	init_props.IC2Filter = dev_cfg->input_filtering_level * LL_TIM_IC_FILTER_FDIV1_N2;

	/* Ensure that the counter will always count up to a multiple of counts_per_revolution */
	if (IS_TIM_32B_COUNTER_INSTANCE(dev_cfg->timer_inst)) {
		max_counter_value = UINT32_MAX - (UINT32_MAX % dev_cfg->counts_per_revolution) - 1;
	} else {
		max_counter_value = UINT16_MAX - (UINT16_MAX % dev_cfg->counts_per_revolution) - 1;
	}
	LL_TIM_SetAutoReload(dev_cfg->timer_inst, max_counter_value);

	if (LL_TIM_ENCODER_Init(dev_cfg->timer_inst, &init_props) != SUCCESS) {
		LOG_ERR("Initalization failed");
		return -EIO;
	}

	LL_TIM_EnableCounter(dev_cfg->timer_inst);

	return 0;
}

static DEVICE_API(sensor, encoder_stm32_driver_api) = {
	.sample_fetch = encoder_stm32_fetch,
	.channel_get = encoder_stm32_get,
};

#define ENCODER_STM32_INIT(n)                                                                         \
	BUILD_ASSERT(!(DT_INST_PROP(n, st_encoder_mode) & ~TIM_SMCR_SMS),                          \
		     "Encoder mode is not supported by this MCU");                                 \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct encoder_stm32_dev_cfg encoder##n##_stm32_config = {                          \
		.pin_config = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                   \
		.timer_inst = ((TIM_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n))),                     \
		.pclken = {.bus = DT_CLOCKS_CELL(DT_INST_PARENT(n), bus),                          \
			   .enr = DT_CLOCKS_CELL(DT_INST_PARENT(n), bits)},                        \
		.encoder_mode = DT_INST_PROP(n, st_encoder_mode),                                  \
		.is_input_polarity_inverted = DT_INST_PROP(n, st_input_polarity_inverted),         \
		.input_filtering_level = DT_INST_PROP(n, st_input_filter_level),                   \
		.counts_per_revolution = DT_INST_PROP(n, st_counts_per_revolution),                \
	};                                                                                         \
                                                                                                   \
	static struct encoder_stm32_dev_data encoder##n##_stm32_data;                                    \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(n, encoder_stm32_initialize, NULL, &encoder##n##_stm32_data,        \
				     &encoder##n##_stm32_config, POST_KERNEL,                         \
				     CONFIG_SENSOR_INIT_PRIORITY, &encoder_stm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ENCODER_STM32_INIT)
