/*
Created as part of "nRF Connect SDK Intermediate" course available on Nordic Dev Academy
*/

#ifndef ZEPHYR_INCLUDE_CUSTOM_MPU6050
#define ZEPHYR_INCLUDE_CUSTOM_MPU6050

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

#define G_VALUE_F   9.80665
#define G_VALUE     980665L

#define MPU6050_GYRO_CONFIG				0x1B
#define MPU6050_ACCEL_CONFIG			0x1C
#define MPU6050_ACCEL_CONFIG2			0x1D
#define MPU6050_LP_ACCEL_ODR			0x1E  

#define MPU6050_REG_ACCEL_XOUT_H		0x3B
#define MPU6050_REG_ACCEL_XOUT_L		0x3C
#define MPU6050_REG_ACCEL_YOUT_H		0x3D
#define MPU6050_REG_ACCEL_YOUT_L		0x3E
#define MPU6050_REG_ACCEL_ZOUT_H		0x3F
#define MPU6050_REG_ACCEL_ZOUT_L		0x40
#define MPU6050_REG_TEMP_OUT_H			0x41
#define MPU6050_REG_TEMP_OUT_L			0x42
#define MPU6050_REG_GYRO_XOUT_H			0x43
#define MPU6050_REG_GYRO_XOUT_L			0x44
#define MPU6050_REG_GYRO_YOUT_H			0x45
#define MPU6050_REG_GYRO_YOUT_L			0x46
#define MPU6050_REG_GYRO_ZOUT_H			0x47
#define MPU6050_REG_GYRO_ZOUT_L			0x48

#define MPU6050_REG_PWR_MGMT_1			0x6B // Device defaults to the SLEEP mode
#define MPU6050_REG_PWR_MGMT_2			0x6C

#define MPU6050_REG_WHO_AM_I            0x75

#define DELAY_VALUES 1000
#define LED0 13

/* Define the driver 'compatible' from the custom device tree binding */
#define DT_DRV_COMPAT zephyr_custom_mpu6050

/* Check if the devicetree contains any devices with the driver compatible */
#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "Custom MPU6050 driver enabled without any devices in DT"
#endif

/* Define data structure to store MPU6050 data */
struct custom_mpu6050_data
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    uint8_t acc_range;
    uint8_t gyro_shift;
    uint8_t chip_id;
};

/* Define data structure to store sensor configuration data */
struct custom_mpu6050_config
{
    struct i2c_dt_spec i2c;
};

#endif /* ZEPHYR_INCLUDE_CUSTOM_MPU6050 */