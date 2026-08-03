#ifndef BSP_IIC_H
#define BSP_IIC_H
#include <stdint.h>
#include "Config_IIC.h"

typedef struct{
    uint8_t SCL_PIN;
    uint8_t SDA_PIN;
}BSP_IIC_t;

typedef struct{
    uint16_t DEVICE_ADDRESS;
}BSP_IIC_DEVICE_t;

extern const BSP_IIC_t BSP_IICS[BSP_IIC_NUM];
extern const BSP_IIC_DEVICE_t BSP_IIC_DEVICES[BSP_IIC_DEVICE_NUM];

#endif