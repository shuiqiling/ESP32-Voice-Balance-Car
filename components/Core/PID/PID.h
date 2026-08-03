#ifndef PID_H
#define PID_H

#include "stdint.h"
#include "Config_ENCODER.h"

float PID_Angle_BalanceCar(float RefenceValue,float CurrentValue,float AngleSpeed);
float PID_Speed_BalanceCar(float RefenceValue, float CurrentValue, BSP_ENCODER_ID_m ENCODER_ID);
float PID_Position_BalanceCar(float RefenceValue, float CurrentValue, BSP_ENCODER_ID_m ENCODER_ID);

/* 在线调参:角度环 / 速度环 / 位置环的 setter */
void PID_Angle_SetKp(float v);
void PID_Angle_SetKi(float v);
void PID_Angle_SetKd(float v);
void PID_Speed_SetKp(float v);
void PID_Speed_SetKi(float v);
void PID_Speed_SetKd(float v);
void PID_Position_SetKp(float v);
void PID_Position_SetKi(float v);
void PID_Position_SetKd(float v);

#endif