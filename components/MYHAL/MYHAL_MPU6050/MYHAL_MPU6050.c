#include "MYHAL_MPU6050.h"
#include "BSP_IIC.h"
#include "IIC.h"
#include "Config_MPU6050_Reg.h"
#include "Config_MPU6050_BITS.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG_MPU = "MPU6050";
#define MPU_FAIL_THRESHOLD 5
static uint8_t  mpu_fail_cnt = 0;

static uint8_t  MYHAL_MPU6050_State[14];
static int16_t  MYHAL_MPU6050_GyroBias[3];   /* [0]=X, [1]=Y, [2]=Z */
static int16_t  MYHAL_MPU6050_GetGyroBias[3] = {0};

/* 异步姿态数据:I2C 任务写,控制环读,mutex 保护 */
static int16_t           MPU6050_AsyncData[6];
static SemaphoreHandle_t mpu_mutex = NULL;

/* 重新配置 MPU6050 寄存器(唤醒+量程+滤波+采样率),不重新 init I2C bus */
static void MPU6050_Reconfig(void){
    uint8_t v;
    v = MPU6050_PWR_WAKE_UP;        IIC_WriteReg(BSP_IIC_MPU6050,MPU6050_PWR_MGMT_1,&v,1);
    v = 0x00;                        IIC_WriteReg(BSP_IIC_MPU6050,MPU6050_PWR_MGMT_2,&v,1);
    v = MPU6050_DLPF_42HZ;           IIC_WriteReg(BSP_IIC_MPU6050,MPU6050_CONFIG,&v,1);
    v = MPU6050_GYRO_FS_250DPS;      IIC_WriteReg(BSP_IIC_MPU6050,MPU6050_GYRO_CONFIG,&v,1);
    v = MPU6050_SMPLRTDIV_200Hz;     IIC_WriteReg(BSP_IIC_MPU6050,MPU6050_SMPLRT_DIV,&v,1);
}

void MYHAL_MPU6050_Init(void){
    IIC_BUS_Init(BSP_IIC_0);
    vTaskDelay(10);
    // IIC_Probe(BSP_IIC_0);   /* 扫描总线,确认 MPU6050 是否在 0x68 响应 */
    IIC_DEVICE_Init(BSP_IIC_0,BSP_IIC_MPU6050);
    MPU6050_Reconfig();
    mpu_fail_cnt = 0;
    if(mpu_mutex == NULL) mpu_mutex = xSemaphoreCreateMutex();
}

/*
 *  陀螺仪零偏校准 —— 调用时芯片必须保持绝对静止
 *  samples: 采样次数，建议 200~500
 *  耗时 ≈ samples × 2ms （例: 200 次约 0.4 秒）
 */
void MYHAL_MPU6050_Calibrate(uint16_t samples)
{
    MYHAL_MPU6050_GyroBias[0] = 0;
    MYHAL_MPU6050_GyroBias[1] = 0;
    MYHAL_MPU6050_GyroBias[2] = 0;
    MYHAL_MPU6050_GetGyroBias[0] = 0;
    MYHAL_MPU6050_GetGyroBias[1] = 0;
    MYHAL_MPU6050_GetGyroBias[2] = 0;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int16_t raw[6] = {0};   /* 初始化,GetData 失败不更新时用 0,避免未初始化使用 */

    for (uint16_t i = 0; i < samples; i++) {
        MYHAL_MPU6050_GetData(raw);
        sum_x += raw[3];   /* GYRO_X */
        sum_y += raw[4];   /* GYRO_Y */
        sum_z += raw[5];   /* GYRO_Z */
        vTaskDelay(2);     /* ~200Hz 采样 */
    }

    MYHAL_MPU6050_GyroBias[0] = (int16_t)(sum_x / samples);
    MYHAL_MPU6050_GyroBias[1] = (int16_t)(sum_y / samples);
    MYHAL_MPU6050_GyroBias[2] = (int16_t)(sum_z / samples);

    printf("GYRO Bias: X=%d Y=%d Z=%d\n",
           MYHAL_MPU6050_GyroBias[0],
           MYHAL_MPU6050_GyroBias[1],
           MYHAL_MPU6050_GyroBias[2]);
}

void MYHAL_MPU6050_GetBias(void)
{
    MYHAL_MPU6050_GetGyroBias[0] = MYHAL_MPU6050_GyroBias[0];
    MYHAL_MPU6050_GetGyroBias[1] = MYHAL_MPU6050_GyroBias[1];
    MYHAL_MPU6050_GetGyroBias[2] = MYHAL_MPU6050_GyroBias[2];
}

void MYHAL_MPU6050_GetData(int16_t *MPU6050DATA){
    esp_err_t ret = IIC_ReadReg(BSP_IIC_MPU6050,MPU6050_ACCEL_XOUT_H,MYHAL_MPU6050_State,14);
    if(ret != ESP_OK){
        /* 读失败:计数,连续失败则恢复总线+重新配置 MPU6050 */
        mpu_fail_cnt++;
        if(mpu_fail_cnt >= MPU_FAIL_THRESHOLD){
            ESP_LOGW(TAG_MPU, "read fail %d times, recovering I2C + MPU6050", mpu_fail_cnt);
            IIC_BusReset(BSP_IIC_0);   /* 总线状态机重置 */
            vTaskDelay(1);
            MPU6050_Reconfig();         /* 重新唤醒+配置 */
            mpu_fail_cnt = 0;
        }
        return;   /* 失败不更新数据,调用方沿用上次值 */
    }
    mpu_fail_cnt = 0;
    MPU6050DATA[ACC_X]  = (int16_t)((MYHAL_MPU6050_State[0] << 8) | MYHAL_MPU6050_State[1]);
    MPU6050DATA[ACC_Y]  = (int16_t)((MYHAL_MPU6050_State[2] << 8) | MYHAL_MPU6050_State[3]);
    MPU6050DATA[ACC_Z]  = (int16_t)((MYHAL_MPU6050_State[4] << 8) | MYHAL_MPU6050_State[5]);
    MPU6050DATA[GYRO_X] = (int16_t)(((MYHAL_MPU6050_State[8] << 8) | MYHAL_MPU6050_State[9])-MYHAL_MPU6050_GetGyroBias[0]);
    MPU6050DATA[GYRO_Y] = (int16_t)(((MYHAL_MPU6050_State[10] << 8) | MYHAL_MPU6050_State[11])-MYHAL_MPU6050_GetGyroBias[1]);
    MPU6050DATA[GYRO_Z] = (int16_t)(((MYHAL_MPU6050_State[12] << 8) | MYHAL_MPU6050_State[13])-MYHAL_MPU6050_GetGyroBias[2]);
}

/* I2C 读取任务:定期同步读 MPU6050,更新共享姿态。
 * I2C 阻塞/失败只卡本任务,不影响控制环。优先级 4 < 控制环 5,会被控制环抢占。*/
static void MPU6050_Task(void *arg){
    static int16_t tmp[6] = {0};   /* static:读失败时保持上次,不让全局跳变 */
    while(1){
        MYHAL_MPU6050_GetData(tmp);   /* 同步读(含失败恢复),成功才更新 tmp */
        xSemaphoreTake(mpu_mutex, portMAX_DELAY);
        memcpy(MPU6050_AsyncData, tmp, sizeof(MPU6050_AsyncData));
        xSemaphoreGive(mpu_mutex);
        vTaskDelay(pdMS_TO_TICKS(5));   /* 200Hz 采样 */
    }
}

/* 启动异步读取任务(在 Calibrate/参考角等同步初始化完成后再调,避免并发读 I2C) */
void MYHAL_MPU6050_StartAsync(void){
    if(mpu_mutex == NULL) mpu_mutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(MPU6050_Task, "mpu_i2c", 4096, NULL, 4, NULL, 1);
}

/* 控制环调这个:非阻塞取最新姿态(mutex 持有极短,不卡控制环) */
void MYHAL_MPU6050_GetDataAsync(int16_t *out){
    if(mpu_mutex == NULL) return;
    xSemaphoreTake(mpu_mutex, portMAX_DELAY);
    memcpy(out, MPU6050_AsyncData, sizeof(MPU6050_AsyncData));
    xSemaphoreGive(mpu_mutex);
}




