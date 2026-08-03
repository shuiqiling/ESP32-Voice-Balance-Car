#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "SER_AIWORK.h"
#include "SER_BALANCECAR.h"
#include "debug_uart.h"

void app_main(void)
{
    dbg_uart_init();       /* 调试串口(UART0 TX=GPIO43),不依赖 USB console */
    SER_BALANCECAR_Init(); /* 平衡车硬件先初始化——不依赖网络,WiFi 连不上也不影响起立 */
    xTaskCreatePinnedToCore(SER_BALANCECAR_Task, "SER_BALANCECAR_Task", 8192, NULL, 7, NULL, 1);
    SER_AIWORK_Init();     /* 网络/音频/AI 初始化(异步,不阻塞平衡车) */
    SER_AIWORK_CreatPin();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
