/*
 * Copyright (c) 2026 jetpax
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Raspberry Pi BCM2835/2710 SoC temperature sensor.
 *
 * The SoC die temperature is read through the VideoCore firmware
 * GET_TEMPERATURE property tag -- the same source as `vcgencmd
 * measure_temp`. The driver owns no MMIO of its own; it just calls
 * bcm2835_property_get_temperature() on the firmware driver.
 * Exposes SENSOR_CHAN_DIE_TEMP.
 */

#define DT_DRV_COMPAT raspberrypi_bcm2835_vc_thermal

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/firmware/bcm2835.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bcm2835_vc_thermal, CONFIG_SENSOR_LOG_LEVEL);

struct bcm2835_vc_thermal_data {
	int32_t millideg;
};

static int bcm2835_vc_thermal_sample_fetch(const struct device *dev,
					   enum sensor_channel chan)
{
	struct bcm2835_vc_thermal_data *data = dev->data;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_DIE_TEMP) {
		return -ENOTSUP;
	}

	return bcm2835_property_get_temperature(&data->millideg);
}

static int bcm2835_vc_thermal_channel_get(const struct device *dev,
					  enum sensor_channel chan,
					  struct sensor_value *val)
{
	const struct bcm2835_vc_thermal_data *data = dev->data;

	if (chan != SENSOR_CHAN_DIE_TEMP) {
		return -ENOTSUP;
	}

	/* Firmware reports millidegrees C; sensor_value is whole degrees
	 * in val1 and millionths of a degree in val2.
	 */
	val->val1 = data->millideg / 1000;
	val->val2 = (data->millideg % 1000) * 1000;

	return 0;
}

static DEVICE_API(sensor, bcm2835_vc_thermal_api) = {
	.sample_fetch = bcm2835_vc_thermal_sample_fetch,
	.channel_get = bcm2835_vc_thermal_channel_get,
};

#define BCM2835_VC_THERMAL_DEFINE(inst)						\
	static struct bcm2835_vc_thermal_data bcm2835_vc_thermal_data_##inst;	\
										\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, NULL, NULL,				\
				     &bcm2835_vc_thermal_data_##inst, NULL,	\
				     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,	\
				     &bcm2835_vc_thermal_api);

DT_INST_FOREACH_STATUS_OKAY(BCM2835_VC_THERMAL_DEFINE)
