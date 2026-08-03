#pragma once

/* Self Test Registers */
#define MPU6050_SELF_TEST_X        0x0D
#define MPU6050_SELF_TEST_Y        0x0E
#define MPU6050_SELF_TEST_Z        0x0F
#define MPU6050_SELF_TEST_A        0x10

/* Sample Rate Divider */
#define MPU6050_SMPLRT_DIV         0x19

/* Configuration */
#define MPU6050_CONFIG             0x1A

/* Gyroscope Configuration */
#define MPU6050_GYRO_CONFIG        0x1B

/* Accelerometer Configuration */
#define MPU6050_ACCEL_CONFIG       0x1C

/* FIFO Enable */
#define MPU6050_FIFO_EN            0x23

/* I2C Master Control */
#define MPU6050_I2C_MST_CTRL       0x24

/* Interrupt */
#define MPU6050_INT_PIN_CFG        0x37
#define MPU6050_INT_ENABLE         0x38
#define MPU6050_INT_STATUS         0x3A

/* Accelerometer Data */
#define MPU6050_ACCEL_XOUT_H       0x3B
#define MPU6050_ACCEL_XOUT_L       0x3C

#define MPU6050_ACCEL_YOUT_H       0x3D
#define MPU6050_ACCEL_YOUT_L       0x3E

#define MPU6050_ACCEL_ZOUT_H       0x3F
#define MPU6050_ACCEL_ZOUT_L       0x40

/* Temperature Data */
#define MPU6050_TEMP_OUT_H         0x41
#define MPU6050_TEMP_OUT_L         0x42

/* Gyroscope Data */
#define MPU6050_GYRO_XOUT_H        0x43
#define MPU6050_GYRO_XOUT_L        0x44

#define MPU6050_GYRO_YOUT_H        0x45
#define MPU6050_GYRO_YOUT_L        0x46

#define MPU6050_GYRO_ZOUT_H        0x47
#define MPU6050_GYRO_ZOUT_L        0x48

/* External Sensor Data */
#define MPU6050_EXT_SENS_DATA_00   0x49

/* User Control */
#define MPU6050_USER_CTRL          0x6A

/* Power Management */
#define MPU6050_PWR_MGMT_1         0x6B
#define MPU6050_PWR_MGMT_2         0x6C

/* FIFO Count */
#define MPU6050_FIFO_COUNTH        0x72
#define MPU6050_FIFO_COUNTL        0x73

/* FIFO Read/Write */
#define MPU6050_FIFO_R_W           0x74

/* Device ID */
#define MPU6050_WHO_AM_I           0x75
