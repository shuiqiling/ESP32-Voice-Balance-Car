#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>
#include "Config_ENCODER.h"


typedef struct {
    uint8_t BSP_ENCODER_A;
    uint8_t BSP_ENCODER_B;
}BSP_ENCODER_t;

extern const BSP_ENCODER_t BSP_ENCODERS[BSP_ENCODER_NUM];

#endif 