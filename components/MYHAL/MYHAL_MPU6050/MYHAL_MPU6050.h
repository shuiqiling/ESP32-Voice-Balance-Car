#ifndef MYHAL_MPU6050_H
#define MYHAL_MPU6050_H

#include "Config_IIC.h"
#include "Config_MPU6050.h"
#include "stdint.h"

void MYHAL_MPU6050_Init(void);
void MYHAL_MPU6050_GetData(int16_t *MPU6050DATA);
void MYHAL_MPU6050_StartAsync(void);            /* 启动异步 I2C 读取任务 */
void MYHAL_MPU6050_GetDataAsync(int16_t *out);  /* 非阻塞取最新姿态(控制环用) */

/**
 * @brief  陀螺仪零偏校准（调用时芯片必须保持静止）
 * @param  samples  采样次数，建议 200~500，大约耗时 samples×2ms
 * @note   校准结果内部保存，通过 MYHAL_MPU6050_GetBias() 获取
 */
void MYHAL_MPU6050_Calibrate(uint16_t samples);

/**
 * @brief  获取校准后的陀螺仪零偏值
 */
void MYHAL_MPU6050_GetBias(void);

#endif