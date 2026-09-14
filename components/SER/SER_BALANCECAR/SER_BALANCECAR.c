#include "SER_BALANCECAR.h"
#include "MYHAL_ENCODER.h"
#include "MYHAL_MOTOR.h"
#include "MYHAL_MPU6050.h"
#include "ComplementaryFilter.h"
#include "PID.h"
#include "math.h"
#include <string.h>
#include "SystemState.h"
#include "esp_log.h"
#include "debug_uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SER_BALANCECAR_SpeedScanGap 1
#define SER_BALANCECAR_PositionScanGap 1
#define SER_BALANCECAR_Encoder_CPR 11
#define SER_BALANCECAR_Encoder_M_To_Cm 100
static const char *TAG = "SER_BALANCECAR";

static int16_t SER_BALANCECAR_MPU6050DATA[6];
static float SER_BALANCECAR_RefenceAngle;
static float MiliPerEncoderCount = 6.80;
static const float SER_BALANCECAR_SCANGAP = 5.0f;   /* 控制周期 ms */

void SER_BALANCECAR_Init(void){
    MYHAL_MOTOR_Init(BSP_RIGHT_MOTOR);
    MYHAL_MOTOR_Init(BSP_LEFT_MOTOR);
    MYHAL_ENCODER_Init(BSP_ENCODER_RIGHT);
    MYHAL_ENCODER_Init(BSP_ENCODER_LEFT);
    MYHAL_MPU6050_Init();
    MYHAL_MPU6050_Calibrate(300);
    MYHAL_MPU6050_GetBias();
    MYHAL_MPU6050_GetData(SER_BALANCECAR_MPU6050DATA);
    SER_BALANCECAR_RefenceAngle = atan2f(SER_BALANCECAR_MPU6050DATA[ACC_X], SER_BALANCECAR_MPU6050DATA[ACC_Z])* 57.29578f;
    /* 启动异步 I2C 读取,之后控制环用 GetDataAsync 非阻塞取姿态 */
    MYHAL_MPU6050_StartAsync();
}

/* 角度环:三环只在 SER_BALANCECAR_Task 单任务中串行调用,无需互斥 */
void SER_BALANCECAR_STAND(float GOATANGLE_R,float GOATANGLE_L){
    MYHAL_MPU6050_GetDataAsync(SER_BALANCECAR_MPU6050DATA);   /* 非阻塞取最新姿态,I2C 卡住不影响控制环 */
    float CurrentAgle = ComplementaryFilter_MPU6050(SER_BALANCECAR_MPU6050DATA, SER_BALANCECAR_SCANGAP / 1000.0f);
    float SER_MOTOR_RSpeed = PID_Angle_BalanceCar(SER_BALANCECAR_RefenceAngle + GOATANGLE_R,CurrentAgle,SER_BALANCECAR_MPU6050DATA[GYRO_Y]);
    float SER_MOTOR_LSpeed = PID_Angle_BalanceCar(SER_BALANCECAR_RefenceAngle + GOATANGLE_L,CurrentAgle,SER_BALANCECAR_MPU6050DATA[GYRO_Y]);

    MYHAL_MOTOR_MOVE(BSP_RIGHT_MOTOR, SER_MOTOR_RSpeed);
    MYHAL_MOTOR_MOVE(BSP_LEFT_MOTOR, SER_MOTOR_LSpeed);
}

/* 速度环:输出为目标倾角增量,喂给角度环 */
void SER_BALANCECAR_SPEED(float RefenceValue_R,float RefenceValue_L){
    static uint16_t SER_BALANCECAR_SpeedCount = SER_BALANCECAR_SpeedScanGap;
    static float SER_BALANCECAR_R_PID_Angle = 0;
    static float SER_BALANCECAR_L_PID_Angle = 0;
    SER_BALANCECAR_SpeedCount --;
    int32_t SER_BALANCECAR_RSpeed = 0;
    int32_t SER_BALANCECAR_LSpeed = 0;
    if(!SER_BALANCECAR_SpeedCount)
    {
        /* 速度 = 编码器增量 × 每计数 mm ÷ 采样周期(cm/s) */
        SER_BALANCECAR_RSpeed = (int32_t)(SER_BALANCECAR_Encoder_M_To_Cm * MYHAL_ENCODER_GetSpeedCounter(BSP_ENCODER_RIGHT) * MiliPerEncoderCount/(SER_BALANCECAR_Encoder_CPR * SER_BALANCECAR_SCANGAP * SER_BALANCECAR_SpeedScanGap));
        SER_BALANCECAR_LSpeed = (int32_t)(SER_BALANCECAR_Encoder_M_To_Cm * MYHAL_ENCODER_GetSpeedCounter(BSP_ENCODER_LEFT) * MiliPerEncoderCount/(SER_BALANCECAR_Encoder_CPR * SER_BALANCECAR_SCANGAP * SER_BALANCECAR_SpeedScanGap));

        SER_BALANCECAR_R_PID_Angle = PID_Speed_BalanceCar(RefenceValue_R,SER_BALANCECAR_RSpeed,BSP_ENCODER_RIGHT);
        SER_BALANCECAR_L_PID_Angle = PID_Speed_BalanceCar(RefenceValue_L,SER_BALANCECAR_LSpeed,BSP_ENCODER_LEFT);

        SER_BALANCECAR_SpeedCount = SER_BALANCECAR_SpeedScanGap;
    }
    SER_BALANCECAR_STAND(SER_BALANCECAR_R_PID_Angle,SER_BALANCECAR_L_PID_Angle);

    /* VOFA 调试:每 5 周期(200/5=40Hz)push 4 个浮点,由 dbg IO 任务异步发送 */
    {static uint8_t d = 0; if(++d >= 5){ d=0;
        float f[] = {SER_BALANCECAR_RSpeed, RefenceValue_R,
                     SER_BALANCECAR_LSpeed, RefenceValue_L};
        dbg_vofa_push(f, 4);
    }}
}

/* 位置环:输出为目标速度,喂给速度环 */
void SER_BALANCECAR_POSITION(float RefenceValue_R,float RefenceValue_L){
    static uint16_t SER_BALANCECAR_PositionCount = SER_BALANCECAR_PositionScanGap;
    static float SER_BALANCECAR_R_PID_Speed = 0;
    static float SER_BALANCECAR_L_PID_Speed = 0;
    SER_BALANCECAR_PositionCount --;
    if(!SER_BALANCECAR_PositionCount)
    {
        /* 位置 = 编码器累计计数 × 每计数 mm(mm) */
        int32_t SER_BALANCECAR_RPosition = (int32_t)(MYHAL_ENCODER_GetCounter(BSP_ENCODER_RIGHT) * MiliPerEncoderCount / SER_BALANCECAR_Encoder_CPR);
        int32_t SER_BALANCECAR_LPosition = (int32_t)(MYHAL_ENCODER_GetCounter(BSP_ENCODER_LEFT) * MiliPerEncoderCount / SER_BALANCECAR_Encoder_CPR);

        SER_BALANCECAR_R_PID_Speed = PID_Position_BalanceCar(RefenceValue_R,SER_BALANCECAR_RPosition,BSP_ENCODER_RIGHT);
        SER_BALANCECAR_L_PID_Speed = PID_Position_BalanceCar(RefenceValue_L,SER_BALANCECAR_LPosition,BSP_ENCODER_LEFT);

        SER_BALANCECAR_PositionCount = SER_BALANCECAR_PositionScanGap;
    }
    SER_BALANCECAR_SPEED(SER_BALANCECAR_R_PID_Speed,SER_BALANCECAR_L_PID_Speed);
}

void SER_BALANCECAR_Task(void *arg){
    static bool busy = false;
    static float target_r = 0, target_l = 0;
    static TickType_t busy_tick = 0;
    const float DONE_THRESH = 5.0f;     /* 位置误差 <5mm 算完成 */
    const int32_t TIMEOUT_MS = 15000;   /* 15秒超时强制完成 */

    while(1){
        /* ① 空闲时取命令(busy 期间不取,保证动作排队的串行)。
         * 串口 pos 优先于 AI:手动调试要能压过语音指令。*/
        ai_cmd_t cmd;
        bool got_cmd = false;

        if (!busy && ser_pos_cmd_pending) {
            /* 取值和清标志必须在同一临界区内,与 dbg_io 任务的写入配对。
             * 先清标志再读值会在跨核场景下读到"新命令的右轮 + 旧命令的左轮"。*/
            portENTER_CRITICAL(&ser_pos_mux);
            target_r = ser_pos_target_r;
            target_l = ser_pos_target_l;
            ser_pos_cmd_pending = false;      /* 先消费再执行,避免同一条命令重复触发 */
            portEXIT_CRITICAL(&ser_pos_mux);
            got_cmd = true;
            ESP_LOGI(TAG, "pos cmd target:(%.0f,%.0f)", target_r, target_l);
        }
        else if (!busy && xQueueReceive(ai_cmd_queue, &cmd, 0) == pdPASS) {
            float cur_mm_r = MYHAL_ENCODER_GetCounter(BSP_ENCODER_RIGHT)
                           * MiliPerEncoderCount / SER_BALANCECAR_Encoder_CPR;
            float cur_mm_l = MYHAL_ENCODER_GetCounter(BSP_ENCODER_LEFT)
                           * MiliPerEncoderCount / SER_BALANCECAR_Encoder_CPR;
            float inc_r = 0, inc_l = 0;

            if      (strcmp(cmd.cmd,"forward")  == 0) { inc_r =  cmd.cmd_value; inc_l = -cmd.cmd_value; }
            else if (strcmp(cmd.cmd,"backward") == 0) { inc_r = -cmd.cmd_value; inc_l =  cmd.cmd_value; }
            else if (strcmp(cmd.cmd,"left")     == 0) { inc_r = -cmd.cmd_value; inc_l = -cmd.cmd_value; }
            else if (strcmp(cmd.cmd,"right")    == 0) { inc_r =  cmd.cmd_value; inc_l =  cmd.cmd_value; }
            /* stop/none → inc=0(原地不动) */

            target_r = cur_mm_r + inc_r;
            target_l = cur_mm_l + inc_l;
            got_cmd = true;
            ESP_LOGI(TAG, "act: %s val=%d target:(%.0f,%.0f)", cmd.cmd, cmd.cmd_value, target_r, target_l);
        }

        if (got_cmd) {
            busy = true;
            busy_tick = xTaskGetTickCount();
        }

        if (busy) {
            SER_BALANCECAR_POSITION(target_r, target_l);
        } else {
            float cur_mm_r = MYHAL_ENCODER_GetCounter(BSP_ENCODER_RIGHT)
                           * MiliPerEncoderCount / SER_BALANCECAR_Encoder_CPR;
            float cur_mm_l = MYHAL_ENCODER_GetCounter(BSP_ENCODER_LEFT)
                           * MiliPerEncoderCount / SER_BALANCECAR_Encoder_CPR;
            SER_BALANCECAR_POSITION(cur_mm_r, cur_mm_l);   /* 空闲时原地保位 */
        }

        /* 急停:busy 期间 ser_emg_stop 为 true → 立即中断 */
        if (busy && ser_emg_stop) {
            busy = false;
            ser_emg_stop = false;
            ESP_LOGW(TAG, "EMERGENCY STOP");
        }

        /* ③ 复位:到达检测 + 超时保护 */
        if (busy) {
            float cur_mm_r = MYHAL_ENCODER_GetCounter(BSP_ENCODER_RIGHT)
                           * MiliPerEncoderCount / SER_BALANCECAR_Encoder_CPR;
            float cur_mm_l = MYHAL_ENCODER_GetCounter(BSP_ENCODER_LEFT)
                           * MiliPerEncoderCount / SER_BALANCECAR_Encoder_CPR;
            bool done = fabsf(cur_mm_r - target_r) < DONE_THRESH
                     && fabsf(cur_mm_l - target_l) < DONE_THRESH;

            if (!done && xTaskGetTickCount() - busy_tick > pdMS_TO_TICKS(TIMEOUT_MS)) {
                done = true;
                ESP_LOGW(TAG, "act timeout, force done");
            }

            if (done) {
                busy = false;
                ESP_LOGI(TAG, "act done: (%.0f,%.0f)", cur_mm_r, cur_mm_l);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SER_BALANCECAR_SCANGAP));
    }
}
