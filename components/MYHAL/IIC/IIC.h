#ifndef IIC_H
#define IIC_H

#include "Config_IIC.h"
#include "stdint.h"
#include "esp_err.h"

void IIC_BUS_Init(BSP_IIC_ID_m IIC_BUS_ID);
void IIC_DEVICE_Init(BSP_IIC_ID_m IIC_BUS_ID,BSP_IIC_DEVICE_m IIC_DEVICE_ID);

esp_err_t IIC_WriteReg(BSP_IIC_DEVICE_m IIC_DEVICE_ID,uint8_t Reg,const uint8_t *Data,uint16_t Len);
esp_err_t IIC_ReadReg(BSP_IIC_DEVICE_m IIC_DEVICE_ID,uint8_t Reg,uint8_t *Data,uint16_t Len);
esp_err_t IIC_BusReset(BSP_IIC_ID_m IIC_BUS_ID);   /* 总线状态机重置(卡 BUSY 时调用) */

#endif