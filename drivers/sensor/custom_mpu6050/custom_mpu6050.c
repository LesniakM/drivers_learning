/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Copyright (c) 2019 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include "custom_mpu6050.h"

LOG_MODULE_REGISTER(custom_mpu6050, CONFIG_SENSOR_LOG_LEVEL);

int mpu6050_reg_read(const struct device *dev, uint8_t reg, uint8_t *data, int size)

{
	const struct custom_mpu6050_config *cfg = dev->config;
	int err;
	// (const struct i2c_dt_spec *spec, uint8_t reg_addr, uint8_t value)
	// WIP
	//err = i2c_reg_write_byte_dt(&cfg->i2c, reg, value);
	if (err)
	{
		LOG_ERR("i2c_reg_write_byte_dt() failed, err %d", err);
		return err;
	}
	return 0;
}

int bme280_reg_read(const struct device *dev,
					uint8_t reg, uint8_t *data, int size)
{
	const struct custom_bme280_config *bus = dev->config;

	uint8_t addr;
	const struct spi_buf tx_spi_buf = {.buf = &addr, .len = 1};
	const struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf rx_buf[2];
	const struct spi_buf_set rx_spi_buf_set = {.buffers = rx_buf, .count = ARRAY_SIZE(rx_buf)};

	int i;
	rx_buf[0].buf = NULL;
	rx_buf[0].len = 1;
	rx_buf[1].len = 1;

	for (i = 0; i < size; i++)
	{
		int err;

		addr = (reg + i) | 0x80;
		rx_buf[1].buf = &data[i];

		err = spi_transceive_dt(&bus->spi, &tx_spi_buf_set, &rx_spi_buf_set);
		if (err)
		{
			LOG_DBG("spi_transceivedt() failed, err: %d", err);
			return err;
		}
	}

	return 0;
}

int mpu6050_reg_write(const struct device *dev, uint8_t reg, uint8_t value)
{
	const struct custom_mpu6050_config *cfg = dev->config;
	int err;
	// (const struct i2c_dt_spec *spec, uint8_t reg_addr, uint8_t value)

	err = i2c_reg_write_byte_dt(&cfg->i2c, reg, value);
	if (err)
	{
		LOG_ERR("i2c_reg_write_byte_dt() failed, err %d", err);
		return err;
	}
	return 0;
}

int mpu6050_wait_until_ready(const struct device *dev)
{
	uint8_t status = 0;
	int ret;

	/* WIP, may not be needed. */
	do
	{
		k_sleep(K_MSEC(3));
		ret = mpu6050_reg_read(dev, REG_STATUS, &status, 1);
		if (ret < 0)
		{
			return ret;
		}
	} while (status & (STATUS_MEASURING | STATUS_IM_UPDATE));

	return 0;
}

static int custom_mpu6050_sample_fetch(const struct device *dev,
									   enum sensor_channel chan)
{
	/* STEP 7.1 - Populate the custom_bme280_sample_fetch() function */
	struct custom_mpu6050_data *data = dev->data;

	uint8_t buf[8];
	int32_t adc_press, adc_temp, adc_humidity;
	int size = 8;
	int err;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);
	// WIP
	//err = bme280_wait_until_ready(dev);
	if (err < 0)
	{
		return err;
	}

	//err = bme280_reg_read(dev, PRESSMSB, buf, size);
	if (err < 0)
	{
		return err;
	}

	return 0;
}
/*
static int custom_bme280_sample_fetch(const struct device *dev,
									  enum sensor_channel chan)
{
	// STEP 7.1 - Populate the custom_bme280_sample_fetch() function 
	struct custom_bme280_data *data = dev->data;

	uint8_t buf[8];
	int32_t adc_press, adc_temp, adc_humidity;
	int size = 8;
	int err;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	err = bme280_wait_until_ready(dev);
	if (err < 0)
	{
		return err;
	}

	err = bme280_reg_read(dev, PRESSMSB, buf, size);
	if (err < 0)
	{
		return err;
	}

	adc_press = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
	adc_temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
	adc_humidity = (buf[6] << 8) | buf[7];

	bme280_compensate_temp(data, adc_temp);
	bme280_compensate_press(data, adc_press);
	bme280_compensate_humidity(data, adc_humidity);

	return 0;
}
*/

static int custom_bme280_channel_get(const struct device *dev,
									 enum sensor_channel chan,
									 struct sensor_value *val)
{
	/* STEP 7.2 - Populate the custom_bme280_channel_get() function */

	return 0;
}

/* STEP 7.3 - Define the sensor driver API */
__subsystem struct sensor_driver_api
{
	sensor_sample_fetch_t sample_fetch;
	sensor_channel_get_t channel_get;
};

static int custom_mpu6050_init(const struct device *dev)
{
	struct custom_6050_data *data = dev->data;
	int err;
	// WIP
	return 0;
}

/* STEP 8 - Define a macro for the device driver instance */

/* STEP 9 - Create the struct device for every status "okay" node in the devicetree */

/*
#define DT_DRV_COMPAT zephyr_example_sensor

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(example_sensor, CONFIG_SENSOR_LOG_LEVEL);

struct example_sensor_data {
	int state;
};

struct example_sensor_config {
	struct gpio_dt_spec input;
};

static int example_sensor_sample_fetch(const struct device *dev,
					  enum sensor_channel chan)
{
	const struct example_sensor_config *config = dev->config;
	struct example_sensor_data *data = dev->data;

	data->state = gpio_pin_get_dt(&config->input);

	return 0;
}

static int example_sensor_channel_get(const struct device *dev,
					 enum sensor_channel chan,
					 struct sensor_value *val)
{
	struct example_sensor_data *data = dev->data;

	if (chan != SENSOR_CHAN_PROX) {
		return -ENOTSUP;
	}

	val->val1 = data->state;

	return 0;
}

static const struct sensor_driver_api example_sensor_api = {
	.sample_fetch = &example_sensor_sample_fetch,
	.channel_get = &example_sensor_channel_get,
};

static int example_sensor_init(const struct device *dev)
{
	const struct example_sensor_config *config = dev->config;

	int ret;

	if (!device_is_ready(config->input.port)) {
		LOG_ERR("Input GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->input, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Could not configure input GPIO (%d)", ret);
		return ret;
	}

	return 0;
}

#define EXAMPLE_SENSOR_INIT(i)						       \
	static struct example_sensor_data example_sensor_data_##i;	       \
										   \
	static const struct example_sensor_config example_sensor_config_##i = {\
		.input = GPIO_DT_SPEC_INST_GET(i, input_gpios),		       \
	};								       \
										   \
	DEVICE_DT_INST_DEFINE(i, example_sensor_init, NULL,		       \
				  &example_sensor_data_##i,			       \
				  &example_sensor_config_##i, POST_KERNEL,	       \
				  CONFIG_SENSOR_INIT_PRIORITY, &example_sensor_api);

DT_INST_FOREACH_STATUS_OKAY(EXAMPLE_SENSOR_INIT)
*/