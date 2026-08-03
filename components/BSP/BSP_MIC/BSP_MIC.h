#ifndef BSP_MIC_H
#define BSP_MIC_H

#include <stdint.h>
#include "Config_MIC.h"



typedef struct {
    uint8_t BSP_MIC_WS;
    uint8_t BSP_MIC_CLK;
    uint8_t BSP_MIC_SD;
}BSP_MIC_t;

extern const BSP_MIC_t BSP_MICS[BSP_MIC_NUM];

#endif 