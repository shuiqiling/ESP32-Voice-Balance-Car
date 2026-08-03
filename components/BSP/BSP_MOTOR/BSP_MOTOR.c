#include "BSP_MOTOR.h"

const BSP_MOTOR_t BSP_MOTORS[BSP_MOTOR_NUM] = {
    [BSP_RIGHT_MOTOR] = {
        .BSP_MOTOR_IN1 = 11,
        .BSP_MOTOR_IN2 = 10,
        .BSP_MOTOR_PWM = 12
    },
    [BSP_LEFT_MOTOR] = {
        .BSP_MOTOR_IN1 = 13,
        .BSP_MOTOR_IN2 = 14,
        .BSP_MOTOR_PWM = 15
    }
};

