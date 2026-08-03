#include "SER_AIWORK.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "AI.h"
#include "WIFI.h"
#include "ASR.h"
#include "MYHAL_MIC.h"

/* 初始化全部异步:WIFI 连不连得上由 ai_task 内部等待,不阻塞本函数,
 * 保证平衡车初始化/起立不被网络阻塞。 */
void SER_AIWORK_Init(void){
    WIFI_Init();           /* 异步连接,ai_task 内部等待拿到 IP 后才开始请求 */
    MYHAL_MIC_Init();      /* 启动 I2S 采集任务，音频流入流缓冲 */
    ASR_Init();            /* 启动 WebSocket(自动重连)+ 队列 */
    AI_Init();             /* 启动 AI 队列 */
}

void SER_AIWORK_CreatPin(void){
     xTaskCreatePinnedToCore(asr_send_task, "asr_send", 8192, NULL, 6, NULL, 0);
     xTaskCreatePinnedToCore(ai_task, "ai_task", 12288, NULL, 5, NULL, 0);
}
