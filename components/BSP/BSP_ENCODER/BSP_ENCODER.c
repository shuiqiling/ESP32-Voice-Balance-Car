#include "BSP_ENCODER.h"

const BSP_ENCODER_t BSP_ENCODERS[BSP_ENCODER_NUM] = {
    [BSP_ENCODER_RIGHT] = {
        .BSP_ENCODER_A = 4,
        .BSP_ENCODER_B = 5
    },
    [BSP_ENCODER_LEFT] = {
        .BSP_ENCODER_A = 16,
        .BSP_ENCODER_B = 17
    }
};
