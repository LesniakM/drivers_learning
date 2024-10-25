#ifndef ZEPHYR_INCLUDE_CUSTOM_MPU6050
#define ZEPHYR_INCLUDE_CUSTOM_MPU6050

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

#define MPU6050_REG_ACCEL_ZOUT_H 0x3f
#define MPU6050_REG_ACCEL_ZOUT_L 0x40
#define MPU6050_REG_WHO_AM_I 0x75
#define MPU6050_REG_PWR_MGMT_1 0x6b
#define MPU6050_REG_ACCEL_CONFIG 0x1c
#define DELAY_REG 10
#define DELAY_PARAM 50
#define DELAY_VALUES 1000
#define LED0 13

/* STEP 4 - Define the driver compatible from the custom binding */
#define DT_DRV_COMPAT zephyr_custom_mpu6050


/* STEP 5 - Check if the devicetree contains any devices with the driver compatible */
#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "Custom MPU6050 driver enabled without any devices"
#endif

/* STEP 6.1 - Define data structure to store BME280 data */
struct custom_mpu6050_data
{
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t temp;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	uint8_t chip_id;
};

/* STEP 6.2 - Define data structure to store sensor configuration data */
struct custom_mpu6050_config
{
	struct i2c_dt_spec i2c;
};

#endif /* ZEPHYR_INCLUDE_CUSTOM_MPU6050 */