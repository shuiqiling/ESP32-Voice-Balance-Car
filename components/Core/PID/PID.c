#include "PID.h"

/* ============ 角度环参数(可在线修改) ============ */
static float PID_Angle_Kp = 13.0f;
static float PID_Angle_Ki = 0.0f;
static float PID_Angle_Kd = 0.9f;
static float PID_Angle_DOUTAEF = 0.80f;  /* 微分低通系数 */

/* ============ 速度环参数 ============ */
static float PID_Speed_Kp = 0.295f;
static float PID_Speed_Ki = 0.001f;
static float PID_Speed_Kd = 0.0f;

/* ============ 位置环参数 ============ */
static float PID_Position_Kp = 0.0004f;
static float PID_Position_Ki = 0.0000002f;
static float PID_Position_Kd = 0.0f;

/* ============ 角度环 ============ */
float PID_Angle_BalanceCar(float RefenceValue, float CurrentValue, float AngleSpeed)
{
    float CurrentError;
    static float PID_Angle_Iout;
    static float PID_Angle_Dout;
    float PID_Angle_out;

    CurrentError = CurrentValue - RefenceValue;

    float Pout = PID_Angle_Kp * CurrentError;
    PID_Angle_Iout += PID_Angle_Ki * CurrentError;
    /* 角度环是唯一不经过上级、直接驱动电机的环,积分必须有界。原来只有这里
     * 没限幅:车被按住/顶住倾角偏置期间输出早已夹在 ±100,而 Iout 还在涨,
     * 松手回正后要靠它自己慢慢退绕 —— 表现就是"松手瞬间满速冲出去"。
     * 这里夹到与输出同量程,保证积分单独就能顶满输出时不再继续累积。
     * 若实测需要更弱的积分权限,把 ±100 收紧即可(速度环用 ±10,位置环 ±20)。*/
    if (PID_Angle_Iout >  100.0f) PID_Angle_Iout =  100.0f;
    if (PID_Angle_Iout < -100.0f) PID_Angle_Iout = -100.0f;

    PID_Angle_Dout = (1.0f - PID_Angle_DOUTAEF) * PID_Angle_Kd * AngleSpeed / 131.0f
                   + PID_Angle_DOUTAEF * PID_Angle_Dout;

    PID_Angle_out = Pout + PID_Angle_Iout - PID_Angle_Dout;
    if (PID_Angle_out >  100.0f) PID_Angle_out =  100.0f;
    if (PID_Angle_out < -100.0f) PID_Angle_out = -100.0f;

    return PID_Angle_out;
}

/* ============ 速度环 ============ */
float PID_Speed_BalanceCar(float RefenceValue, float CurrentValue, BSP_ENCODER_ID_m ENCODER_ID)
{
    float CurrentError;
    static float LastError[BSP_ENCODER_NUM];
    static float PID_Speed_Iout[BSP_ENCODER_NUM];
    float PID_Speed_out;

    CurrentError = RefenceValue - CurrentValue;

    float Pout = PID_Speed_Kp * CurrentError;
    PID_Speed_Iout[ENCODER_ID] += PID_Speed_Ki * CurrentError;
    if (PID_Speed_Iout[ENCODER_ID] >  10.0f) PID_Speed_Iout[ENCODER_ID] =  10.0f;
    if (PID_Speed_Iout[ENCODER_ID] < -10.0f) PID_Speed_Iout[ENCODER_ID] = -10.0f;

    float Dout = PID_Speed_Kd * (CurrentError - LastError[ENCODER_ID]);
    if (Dout >  0.2f) Dout =  0.2f;
    if (Dout < -0.2f) Dout = -0.2f;

    PID_Speed_out = Pout + PID_Speed_Iout[ENCODER_ID] - Dout;
    if (PID_Speed_out >  100.0f) PID_Speed_out =  100.0f;
    if (PID_Speed_out < -100.0f) PID_Speed_out = -100.0f;

    LastError[ENCODER_ID] = CurrentError;
    return PID_Speed_out;
}

/* ============ 位置环 ============ */
float PID_Position_BalanceCar(float RefenceValue, float CurrentValue, BSP_ENCODER_ID_m ENCODER_ID)
{
    float CurrentError;
    static float LastError[BSP_ENCODER_NUM];
    static float PID_Position_Iout[BSP_ENCODER_NUM];
    float PID_Position_out;

    CurrentError = RefenceValue - CurrentValue;

    float Pout = PID_Position_Kp * CurrentError;
    PID_Position_Iout[ENCODER_ID] += PID_Position_Ki * CurrentError;
    if (PID_Position_Iout[ENCODER_ID] >  20.0f) PID_Position_Iout[ENCODER_ID] =  20.0f;
    if (PID_Position_Iout[ENCODER_ID] < -20.0f) PID_Position_Iout[ENCODER_ID] = -20.0f;

    float Dout = PID_Position_Kd * (CurrentError - LastError[ENCODER_ID]);

    PID_Position_out = Pout + PID_Position_Iout[ENCODER_ID] + Dout;
    if (PID_Position_out >  100.0f) PID_Position_out =  100.0f;
    if (PID_Position_out < -100.0f) PID_Position_out = -100.0f;

    LastError[ENCODER_ID] = CurrentError;
    return PID_Position_out;
}

/* ============ 在线调参 setter ============ */
void PID_Angle_SetKp(float v){ PID_Angle_Kp = v; }
void PID_Angle_SetKi(float v){ PID_Angle_Ki = v; }
void PID_Angle_SetKd(float v){ PID_Angle_Kd = v; }

void PID_Speed_SetKp(float v){ PID_Speed_Kp = v; }
void PID_Speed_SetKi(float v){ PID_Speed_Ki = v; }
void PID_Speed_SetKd(float v){ PID_Speed_Kd = v; }

void PID_Position_SetKp(float v){ PID_Position_Kp = v; }
void PID_Position_SetKi(float v){ PID_Position_Ki = v; }
void PID_Position_SetKd(float v){ PID_Position_Kd = v; }