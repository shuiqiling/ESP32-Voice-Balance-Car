#include "BSP_IIC.h"

const BSP_IIC_t BSP_IICS[BSP_IIC_NUM] = {
    [BSP_IIC_0] = {
        .SCL_PIN = 8,
        .SDA_PIN = 9
    }
};

const BSP_IIC_DEVICE_t BSP_IIC_DEVICES[BSP_IIC_DEVICE_NUM] = {
    [BSP_IIC_MPU6050] = {
        .DEVICE_ADDRESS = 0x68
    }
};









