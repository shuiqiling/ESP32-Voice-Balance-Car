#include "IIC.h"
#include "driver/i2c_master.h"
#include "BSP_IIC.h"
#include <string.h>
#include <stdio.h>

#define IIC_TIMEOUT_MS 200
#define IIC_MAX_WRITE_LEN 16   /* 单次写寄存器最大字节数(栈上缓冲上限) */

static i2c_master_bus_handle_t IIC_BUS_HANDLE[BSP_IIC_NUM];
static i2c_master_dev_handle_t IIC_DEVICE_HANDLE[BSP_IIC_DEVICE_NUM];
static BSP_IIC_ID_m i2c_dev_bus[BSP_IIC_DEVICE_NUM];
static bool i2c_dev_inited[BSP_IIC_DEVICE_NUM];

void IIC_BUS_Init(BSP_IIC_ID_m IIC_BUS_ID){

    const i2c_master_bus_config_t conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.allow_pd = 0,
        .flags.enable_internal_pullup = 1,   /* 开内部上拉,外部无上拉时可救命 */
        .glitch_ignore_cnt = 7,
        .i2c_port = IIC_BUS_ID,
        .intr_priority = 0,
        .scl_io_num = BSP_IICS[IIC_BUS_ID].SCL_PIN,
        .sda_io_num = BSP_IICS[IIC_BUS_ID].SDA_PIN,
        .trans_queue_depth = 0
    };

    esp_err_t ret;
    ret = i2c_new_master_bus(&conf, &IIC_BUS_HANDLE[IIC_BUS_ID]);
    if(ret != ESP_OK){
        printf("I2C BUS INIT FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
}

void IIC_DEVICE_Init(BSP_IIC_ID_m IIC_BUS_ID,BSP_IIC_DEVICE_m IIC_DEVICE_ID){

    if(!IIC_BUS_HANDLE[IIC_BUS_ID]){
        printf("I2C DEVICE INIT FAIL: bus not initialized\n");
        return;
    }

    const i2c_device_config_t conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BSP_IIC_DEVICES[IIC_DEVICE_ID].DEVICE_ADDRESS,
        .flags.disable_ack_check = 0,
        .scl_speed_hz = 100000,
        .scl_wait_us = 0
    };

    esp_err_t ret;
    ret = i2c_master_bus_add_device(IIC_BUS_HANDLE[IIC_BUS_ID], &conf, &IIC_DEVICE_HANDLE[IIC_DEVICE_ID]);
    if(ret != ESP_OK){
        printf("I2C DEVICE INIT FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
    i2c_dev_bus[IIC_DEVICE_ID] = IIC_BUS_ID;
    i2c_dev_inited[IIC_DEVICE_ID] = true;
}

esp_err_t IIC_WriteReg(BSP_IIC_DEVICE_m IIC_DEVICE_ID,uint8_t Reg,const uint8_t *Data,uint16_t Len){

    if(!i2c_dev_inited[IIC_DEVICE_ID]){
        printf("I2C WRITE FAIL: device not initialized\n");
        return ESP_ERR_INVALID_STATE;
    }
    if(Len > IIC_MAX_WRITE_LEN){
        printf("I2C WRITE FAIL: len %u > %d\n", (unsigned)Len, IIC_MAX_WRITE_LEN);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[1 + IIC_MAX_WRITE_LEN];   /* 固定上限,不用 VLA,防栈溢出 */
    buf[0] = Reg;
    memcpy(buf + 1, Data, Len);

    esp_err_t ret;
    ret = i2c_master_transmit(IIC_DEVICE_HANDLE[IIC_DEVICE_ID], buf, 1 + Len, IIC_TIMEOUT_MS);
    if(ret != ESP_OK){
        printf("I2C WRITE REG 0x%02X FAIL: %s\n", Reg, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t IIC_ReadReg(BSP_IIC_DEVICE_m IIC_DEVICE_ID,uint8_t Reg,uint8_t *Data,uint16_t Len){

    if(!i2c_dev_inited[IIC_DEVICE_ID]){
        printf("I2C READ FAIL: device not initialized\n");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;
    ret = i2c_master_transmit_receive(IIC_DEVICE_HANDLE[IIC_DEVICE_ID], &Reg, 1, Data, Len, IIC_TIMEOUT_MS);
    if(ret != ESP_OK){
        printf("I2C READ REG 0x%02X FAIL: %s\n", Reg, esp_err_to_name(ret));
    }
    return ret;
}

/* 重置 I2C 总线状态机(总线卡 BUSY 时调用,让 state 回 READY) */
esp_err_t IIC_BusReset(BSP_IIC_ID_m IIC_BUS_ID){
    if(!IIC_BUS_HANDLE[IIC_BUS_ID]){
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_bus_reset(IIC_BUS_HANDLE[IIC_BUS_ID]);
}
