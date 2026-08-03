#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>
#include "Config_MOTOR.h"



typedef struct {
    uint8_t BSP_MOTOR_PWM;
    uint8_t BSP_MOTOR_IN1;
    uint8_t BSP_MOTOR_IN2;
}BSP_MOTOR_t;

extern const BSP_MOTOR_t BSP_MOTORS[BSP_MOTOR_NUM];

#endif 