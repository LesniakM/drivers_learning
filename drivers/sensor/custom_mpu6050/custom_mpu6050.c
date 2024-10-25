/*
Created as part of "nRF Connect SDK Intermediate" course available on Nordic Dev Academy
*/

#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include "custom_mpu6050.h"

LOG_MODULE_REGISTER(custom_mpu6050, CONFIG_SENSOR_LOG_LEVEL);

int mpu6050_burst_read(const struct device *dev, uint8_t start_reg, uint8_t *data, int size)
{
	const struct custom_mpu6050_config *cfg = dev->config;
	int err = i2c_burst_read_dt(&cfg->i2c, start_reg, &data, size);
	if (err)
	{
		LOG_ERR("mpu6050_reg_read/i2c_burst_read_dt() failed, err %d", err);
		return err;
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

/**
 * @brief Fetch a sample from the sensor and store it in an internal
 * driver buffer. Works for both sensor_sample_fetch_chan() and sensor_sample_fetch()
 *
 * Since the function communicates with the sensor device, it is unsafe
 * to call it in an ISR if the device is connected via I2C or SPI.
 *
 * @param dev Pointer to the sensor device
 * @param chan The channel that needs updated
 *
 * @return 0 if successful, negative errno code if failure.
 */
static int custom_mpu6050_sample_fetch(const struct device *dev,
									   enum sensor_channel chan)
{
	const struct custom_mpu6050_config *cfg = dev->config;
	struct custom_mpu6050_data *data = dev->data;

	uint8_t start_address = 0;
	uint8_t end_address = 0;
	switch (chan)
	{
	case SENSOR_CHAN_ALL:
		start_address = MPU6050_REG_ACCEL_XOUT_H;
		end_address = MPU6050_REG_GYRO_ZOUT_L;
		break;
	case SENSOR_CHAN_ACCEL_Z:
		start_address = MPU6050_REG_ACCEL_ZOUT_H;
		end_address = MPU6050_REG_ACCEL_ZOUT_L;
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		start_address = MPU6050_REG_ACCEL_XOUT_H;
		end_address = MPU6050_REG_ACCEL_ZOUT_L;
		break;
	default:
		LOG_ERR("custom_mpu6050_sample_fetch() - Channel %d not implemented!", chan);
		return -999;
	}

	/* Alternatively static buffer of max size can be used instead of VLA*/
	uint8_t buffer[end_address - start_address + 1];

	int err = mpu6050_burst_read(&cfg->i2c, start_address, buffer, sizeof(buffer));
	if (err < 0)
	{
		LOG_ERR("custom_mpu6050_sample_fetch() failed, err %d", err);
		return err;
	}
	switch (chan)
	{
	case SENSOR_CHAN_ALL:
		data->accel_x = (buffer[0] << 8) + buffer[1];
		data->accel_y = (buffer[2] << 8) + buffer[3];
		data->accel_z = (buffer[4] << 8) + buffer[5];
		data->temp = (buffer[6] << 8) + buffer[7];
		data->gyro_x = (buffer[8] << 8) + buffer[9];
		data->gyro_y = (buffer[10] << 8) + buffer[11];
		data->gyro_z = (buffer[12] << 8) + buffer[13];
		break;
	case SENSOR_CHAN_ACCEL_Z:
		data->accel_z = (buffer[0] << 8) + buffer[1];
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		data->accel_x = (buffer[0] << 8) + buffer[1];
		data->accel_y = (buffer[2] << 8) + buffer[3];
		data->accel_z = (buffer[4] << 8) + buffer[5];
		break;
	default:
		LOG_ERR("custom_mpu6050_sample_fetch() - Channel %d not implemented!", chan);
		return -999;
	}

	return 0;
}

/*
From MPU6050 datasheet:
AFS_SEL Full Scale Range LSB Sensitivity
0	±2g 	16384 	LSB/g
1	±4g		8192 	LSB/g
2	±8g 	4096 	LSB/g
3	±16g 	2048 	LSB/g

Data is stored as 16 bit int (32768 if 16 bits max is 65535)
Zephyr sensor API require acceleration in m/s^2, stored as
 val1 . val2

Such as:
0.5: 	val1 =  0, val2 =  500000
-0.5: 	val1 =  0, val2 = -500000
-1.0: 	val1 = -1, val2 =  0
-1.5: 	val1 = -1, val2 = -500000
*/
static void convert_accel(struct sensor_value *val, int16_t raw_val, uint8_t g_range)
{
	int32_t converted_val = ((int32_t)raw_val * G_VALUE) >> (15 - __builtin_ctz(g_range));
	printf("\nVal1 %d  Val2 %d\n", converted_val / 1000000, converted_val % 1000000);

	val->val1 = converted_val / 1000000;
	val->val2 = converted_val % 1000000;
}

/*
From MPU6050 datasheet:
FS_SEL Full Scale Range LSB Sensitivity
0 ± 250 °/s 131 LSB/°/s     131072  (2^17)
1 ± 500 °/s 65.5 LSB/°/s    65 536  (2^16)
2 ± 1000 °/s 32.8 LSB/°/s   32 768  (2^15)
3 ± 2000 °/s 16.4 LSB/°/s   16 384  (2^14)

Return result in degree, which is not compliant with Zephyr sensor API!
*/
static void convert_gyro(struct sensor_value *val, int16_t raw_val, uint8_t gyro_shift)
{
	int32_t converted_val = ((int64_t)raw_val * 1000000000) >> (17 - gyro_shift);
	val->val1 = converted_val / 1000000;
	val->val2 = converted_val % 1000000;
}

/*
Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
*/
static void convert_temp(struct sensor_value *val, int16_t raw_val, uint8_t g_range)
{
	int32_t converted_val = raw_val*100000 / 34 + 36530000;
	val->val1 = converted_val / 1000000;
	val->val2 = converted_val % 1000000;
}

static int custom_mpu6050_channel_get(const struct device *dev,
									  enum sensor_channel chan,
									  struct sensor_value *val)
{
	/* STEP 7.2 - Populate the custom_bme280_channel_get() function */
	struct custom_mpu6050_data *data = dev->data;

	switch (chan)
	{
	case SENSOR_CHAN_ALL:
		convert_accel(val, data->accel_x, data->acc_range);
		convert_accel(val + 1, data->accel_y, data->acc_range);
		convert_accel(val + 2, data->accel_z, data->acc_range);
		convert_temp(val + 12, data->temp, data->acc_range);
		convert_gyro(val + 4, data->gyro_x, data->acc_range);
		convert_gyro(val + 5, data->gyro_y, data->acc_range);
		convert_gyro(val + 6, data->gyro_z, data->acc_range);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		convert_accel(val, data->accel_z, data->acc_range);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		convert_accel(val, data->accel_x, data->acc_range);
		convert_accel(val + 1, data->accel_y, data->acc_range);
		convert_accel(val + 2, data->accel_z, data->acc_range);
		break;
	case SENSOR_CHAN_GYRO_X:
		convert_gyro(val, data->gyro_x, data->acc_range);
		break;
	case SENSOR_CHAN_GYRO_Y:
		convert_gyro(val, data->gyro_y, data->acc_range);
		break;
	default:
		LOG_ERR("custom_mpu6050_channel_get() - Channel %d not implemented!", chan);
		return -999;
	}
	return 0;
}

/* STEP 7.3 - Define the sensor driver API */
static const struct sensor_driver_api custom_mpu6050_api = {
	.sample_fetch = &custom_mpu6050_sample_fetch,
	.channel_get = &custom_mpu6050_channel_get,
};
__subsystem struct sensor_driver_api
{
	sensor_sample_fetch_t sample_fetch;
	sensor_channel_get_t channel_get;
};

static int custom_mpu6050_init(const struct device *dev)
{
	struct custom_mpu6050_data *data = dev->data;
	const struct custom_mpu6050_config *cfg = dev->config;

	data->acc_range = 2;
	data->gyro_shift = 0;

	if (!device_is_ready(&cfg->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	mpu6050_reg_write(&cfg->i2c, MPU6050_REG_PWR_MGMT_1, 0);
	k_msleep(1);
	
	return 0;
}


/* STEP 8 - Define a macro for the device driver instance */
#define CUSTOM_MPU6050_DEFINE(inst)												\
	static struct custom_mpu6050_data custom_mpu6050_data_##inst;					\
	static const struct custom_mpu6050_config custom_mpu6050_config_##inst = {	\
		.i2c = I2C_DT_SPEC_INST_GET(inst),								\
	};  																		\
	DEVICE_DT_INST_DEFINE(inst,													\
				custom_mpu6050_init,												\
				NULL,															\
				&custom_mpu6050_data_##inst,										\
				&custom_mpu6050_config_##inst,									\
				POST_KERNEL, 													\
				CONFIG_SENSOR_INIT_PRIORITY, 									\
				&custom_mpu6050_api);

/* STEP 9 - Create the struct device for every status "okay" node in the devicetree */
DT_INST_FOREACH_STATUS_OKAY(CUSTOM_MPU6050_DEFINE)
/*
#define DT_DRV_COMPAT zephyr_example_sensor


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