#ifndef _COMPLEMENTARYFILTER_H
#define _COMPLEMENTARYFILTER_H

#include "stdint.h"
#include "Config_MPU6050.h"

/**
 * @brief  互补滤波融合 MPU6050 加速度计与陀螺仪,得到俯仰角
 * @param  MPU6050_Data 6 元素姿态原始数据(下标见 Config_MPU6050.h)
 * @param  dt_s         控制周期(秒),须与控制环实际周期一致
 * @return 俯仰角(度)
 */
float ComplementaryFilter_MPU6050(int16_t *MPU6050_Data, float dt_s);

#endif
