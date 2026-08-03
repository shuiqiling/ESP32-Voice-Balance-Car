#pragma once

/* PWR_MGMT_1 电源寄存器*/
#define MPU6050_PWR_WAKE_UP   0x00
#define MPU6050_DEVICE_RESET       (1 << 7)
#define MPU6050_SLEEP              (1 << 6)
#define MPU6050_CYCLE              (1 << 5)
#define MPU6050_TEMP_DIS           (1 << 3)

/* GYRO_CONFIG 陀螺仪配置寄存器*/
#define MPU6050_GYRO_FS_250DPS     (0 << 3)
#define MPU6050_GYRO_FS_500DPS     (1 << 3)
#define MPU6050_GYRO_FS_1000DPS    (2 << 3)
#define MPU6050_GYRO_FS_2000DPS    (3 << 3)

/* ACCEL_CONFIG 加速度计配置寄存器*/
#define MPU6050_ACCEL_FS_2G        (0 << 3)
#define MPU6050_ACCEL_FS_4G        (1 << 3)
#define MPU6050_ACCEL_FS_8G        (2 << 3)
#define MPU6050_ACCEL_FS_16G       (3 << 3)

/* CONFIG(DLPF) 低通滤波器*/
#define MPU6050_DLPF_256HZ         0
#define MPU6050_DLPF_188HZ         1
#define MPU6050_DLPF_98HZ          2
#define MPU6050_DLPF_42HZ          3
#define MPU6050_DLPF_20HZ          4
#define MPU6050_DLPF_10HZ          5
#define MPU6050_DLPF_5HZ           6

//采样率设置
#define MPU6050_SMPLRTDIV_200Hz    4
