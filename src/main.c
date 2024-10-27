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
	struct sensor_value acc_x_val, acc_y_val, acc_z_val;
	struct sensor_value gyro_x_val, gyro_y_val, gyro_z_val;
	struct sensor_value temp_val;

	err = device_is_ready(dev);
	if (!err)
	{
		LOG_INF("Error: I2C device is not ready, err: %d", err);
		return 0;
	}
	LOG_INF("Device is ready!");
	while (1)
	{
		/* Continuously read out sensor data using the sensor API calls */
		LOG_INF("Sample fetch...");
		err = sensor_sample_fetch(dev);
		LOG_INF("Got the sample!");
		if (err < 0)
		{
			LOG_ERR("Could not fetch sample (%d)", err);
			return 0;
		}
		/* Convert acc on Z axis*/
		if (sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &acc_z_val))
		{
			LOG_ERR("Could not get acc Z sample");
			//return 0;
		}
		/* Convert gyro on Z axis*/
		if (sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gyro_z_val))
		{
			LOG_ERR("Could not get gyro Z sample");
			//return 0;
		}
		/* Convert temperature*/
		if (sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temp_val))
		{
			LOG_ERR("Could not get temp sample");
			//return 0;
		}

		LOG_INF("Acc z values: %d.%d", acc_z_val.val1, acc_z_val.val2);
		LOG_INF("Gyro z values: %d.%d", gyro_z_val.val1, gyro_z_val.val2);
		LOG_INF("Temperature values: %d.%d *C", temp_val.val1, temp_val.val2);
		k_sleep(K_MSEC(3000));
	}

	return 0;
}