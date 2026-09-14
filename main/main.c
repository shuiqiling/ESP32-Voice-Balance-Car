#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "SER_AIWORK.h"
#include "SER_BALANCECAR.h"
#include "debug_uart.h"

void app_main(void)
{
    /* 初始化顺序:平衡车硬件在前——它不依赖网络,WiFi 连不上也要能起立。
     * SER_AIWORK_Init 内部全部异步(WiFi 连不连得上由 ai_task 自己等),
     * 所以放后面也不会拖慢控制环启动。*/
    dbg_uart_init();       /* 调试串口 + VOFA 帧,不依赖 USB console */
    SER_BALANCECAR_Init();
    SER_AIWORK_Init();     /* 网络/音频/AI 初始化(异步) */

    /* 控制环绑核心 1、优先级 7,高于同核的 mpu_i2c(4),保证 5ms 周期不被抢占 */
    xTaskCreatePinnedToCore(SER_BALANCECAR_Task, "SER_BALANCECAR_Task",
                            8192, NULL, 7, NULL, 1);

    /* 网络侧任务由 SER_AIWORK 建在核心 0,与控制环分核 */
    SER_AIWORK_CreatPin();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
