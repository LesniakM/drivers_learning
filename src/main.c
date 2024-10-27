/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(Lesson7_Exercise1, LOG_LEVEL_INF);

#define MPU6050_NODE DT_NODELABEL(mpu6050)  // Get the node label for the MPU6050

int main(void)
{	
	const struct device *dev = DEVICE_DT_GET(MPU6050_NODE);
	int err;
	struct sensor_value values[7];
	
	err = device_is_ready(dev);
	if (!err)
	{
		LOG_INF("Error: I2C device is not ready, err: %d", err);
		return 0;
	}
	LOG_INF("Device is ready!");

	sensor_channel_get(dev, SENSOR_CHAN_ALL, values);

	while (1)
	{
		/* Continuously read out sensor data using the sensor API calls */
		err = sensor_sample_fetch(dev);
		if (err < 0)
		{
			LOG_ERR("Could not fetch sample (%d)", err);
			return 0;
		}
		/* Convert all channels*/
		if (sensor_channel_get(dev, SENSOR_CHAN_ALL, values))
		{
			LOG_ERR("Could not get sample");
			//return 0;
		}

		LOG_INF("ACC: X: %d.%d, Y: %d.%d, Z: %d.%d", values[0].val1, values[0].val2, values[1].val1, values[1].val2, values[2].val1, values[2].val2);
		LOG_INF("GYR: X: %d.%d, Y: %d.%d, Z: %d.%d", values[4].val1, values[4].val2, values[5].val1, values[5].val2, values[6].val1, values[6].val2);
		LOG_INF("Temperature: %d.%d *C", values[3].val1, values[3].val2);
		k_sleep(K_MSEC(3000));
	}

	return 0;
}