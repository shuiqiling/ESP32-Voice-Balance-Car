#include "ComplementaryFilter.h"
#include <math.h>

#define GYRO_SCALE   131.0f   /* ±250dps 灵敏度 LSB/(°/s) */
#define ALPHA        0.98f    /* 互补滤波系数 */

/* dt_s:控制周期(秒),由调用方传入,避免与控制环周期隐式耦合 */
float ComplementaryFilter_MPU6050(int16_t *MPU6050_Data, float dt_s)
{
    static int8_t Com_MPU6050_Init_Count = 0;
    static float  Com_MPU6050_Angle = 0.0f;

    float ACC_Angle;
    float GYRO_Rate;

    if (!Com_MPU6050_Init_Count) {
        Com_MPU6050_Init_Count++;
        Com_MPU6050_Angle = atan2f(MPU6050_Data[ACC_X], MPU6050_Data[ACC_Z]) * 57.29578f;
        return Com_MPU6050_Angle;
    }

    ACC_Angle = atan2f(MPU6050_Data[ACC_X], MPU6050_Data[ACC_Z]) * 57.29578f;
    GYRO_Rate = -MPU6050_Data[GYRO_Y] / GYRO_SCALE;

    Com_MPU6050_Angle = ALPHA * (Com_MPU6050_Angle + GYRO_Rate * dt_s)
                      + (1.0f - ALPHA) * ACC_Angle;

    return Com_MPU6050_Angle;
}
