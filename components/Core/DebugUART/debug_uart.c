#include "debug_uart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "PID.h"
#include "SystemState.h"

#define DBG_UART_PORT    UART_NUM_0
#define DBG_UART_TX_PIN  43       /* ESP32-S3 开发板 CH340 的 UART0 TX(GPIO43) */
#define DBG_UART_RX_PIN  44       /* ESP32-S3 开发板 CH340 的 UART0 RX(GPIO44) */
#define DBG_UART_BAUD    115200

#define DBG_IO_TASK_STACK 3072
#define DBG_IO_TASK_PRIO  1       /* 低优先级:串口收发不抢占控制环 */

/* ===== VOFA 生产者/消费者环形缓冲 =====
 * 控制环(高优先级)只做入队(db_vofa_push,非阻塞、满则丢),
 * 由低优先级 IO 任务消费发送,控制环不接触 uart 硬件。 */
#define DBG_VOFA_SLOTS   8
#define DBG_VOFA_MAXF    8

typedef struct {
    float data[DBG_VOFA_MAXF];
    int   count;
} vofa_slot_t;

static vofa_slot_t vofa_ring[DBG_VOFA_SLOTS];
static volatile int vofa_head = 0;   /* 写游标(生产者) */
static volatile int vofa_tail = 0;   /* 读游标(消费者) */

static bool dbg_inited = false;   /* dbg_uart_init() 调用后才为 true */

/* 生产:非阻塞入队,环形满则丢当前帧(控制环不允许等待 uart) */
void dbg_vofa_push(const float *data, int count)
{
    if (!dbg_inited || count <= 0 || count > DBG_VOFA_MAXF) return;
    int next = (vofa_head + 1) % DBG_VOFA_SLOTS;
    if (next == vofa_tail) return;   /* 环满,丢弃(消费端 20ms 一轮,很少发生) */
    memcpy(vofa_ring[vofa_head].data, data, count * sizeof(float));
    vofa_ring[vofa_head].count = count;
    vofa_head = next;
}

/* 消费:取一帧,非空返回 true */
static bool dbg_vofa_pop(vofa_slot_t *out)
{
    if (vofa_tail == vofa_head) return false;
    *out = vofa_ring[vofa_tail];
    vofa_tail = (vofa_tail + 1) % DBG_VOFA_SLOTS;
    return true;
}

void dbg_uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = DBG_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(DBG_UART_PORT, 512, 512, 0, NULL, 0);  /* TX+RX 各 512B */
    uart_param_config(DBG_UART_PORT, &cfg);
    uart_set_pin(DBG_UART_PORT, DBG_UART_TX_PIN, DBG_UART_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    dbg_inited = true;

    /* 独立 IO 任务:VOFA 帧发送 + 串口命令解析,避免阻塞控制环 */
    if (xTaskCreate(dbg_io_task, "dbg_io", DBG_IO_TASK_STACK, NULL,
                    DBG_IO_TASK_PRIO, NULL) != pdPASS) {
        ESP_LOGE("DBG", "dbg_io task create failed");
    }
}

/* 以下函数未 init 时静默跳过,不崩 */
int dbg_available(void)
{
    if (!dbg_inited) return 0;
    int len = 0;
    uart_get_buffered_data_len(DBG_UART_PORT, (size_t *)&len);
    return len;
}

int dbg_read(uint8_t *buf, int max_len)
{
    if (!dbg_inited) return 0;
    int avail = dbg_available();
    if (avail <= 0) return 0;
    if (avail > max_len) avail = max_len;
    return uart_read_bytes(DBG_UART_PORT, buf, avail, 0);
}

void dbg_write(const char *s)
{
    if (!dbg_inited) return;
    uart_write_bytes(DBG_UART_PORT, s, strlen(s));
}

void dbg_printf(const char *fmt, ...)
{
    if (!dbg_inited) return;
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    uart_write_bytes(DBG_UART_PORT, buf, strlen(buf));
}

static void dbg_vofa_send(const float *data, int count)
{
    if (!dbg_inited || count <= 0) return;
    char buf[256];
    int pos = 0;
    for (int i = 0; i < count; i++) {
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%f%s",
                        data[i], (i == count - 1) ? "\n" : ",");
    }
    uart_write_bytes(DBG_UART_PORT, buf, pos);
}

/* 串口命令解析 → 在线调 PID 参数
 * 格式: "akp 25\n" → 角度环Kp, "skp 0.2\n" → 速度环Kp,
 *       "akd 1.5\n" → 角度环Kd, "pos 100 -100\n" → 目标位置(mm)
 */
void dbg_process_cmd(const uint8_t *data, int len)
{
    char line[64];
    if (len > (int)sizeof(line) - 1) len = (int)sizeof(line) - 1;
    memcpy(line, data, len);
    line[len] = '\0';

    /* 去掉末尾换行 */
    char *e = strpbrk(line, "\r\n");
    if (e) *e = '\0';

    float val = 0.0f;
    char cmd[8] = {0};
    sscanf(line, "%7s %f", cmd, &val);

    if      (strcmp(cmd, "akp") == 0) PID_Angle_SetKp(val);
    else if (strcmp(cmd, "aki") == 0) PID_Angle_SetKi(val);
    else if (strcmp(cmd, "akd") == 0) PID_Angle_SetKd(val);
    else if (strcmp(cmd, "skp") == 0) PID_Speed_SetKp(val);
    else if (strcmp(cmd, "ski") == 0) PID_Speed_SetKi(val);
    else if (strcmp(cmd, "skd") == 0) PID_Speed_SetKd(val);
    else if (strcmp(cmd, "pkp") == 0) PID_Position_SetKp(val);
    else if (strcmp(cmd, "pki") == 0) PID_Position_SetKi(val);
    else if (strcmp(cmd, "pkd") == 0) PID_Position_SetKd(val);
    else if (strcmp(cmd, "pos") == 0) {
        float r = 0, l = 0;
        sscanf(line, "pos %f %f", &r, &l);  /* "pos 100 -100" → 右100mm 左-100mm */
        ser_pos_target_r = r;
        ser_pos_target_l = l;
        dbg_printf("pos: R=%.0f L=%.0f\n", r, l);
    }
    else if (strcmp(cmd, "info") == 0) {
        dbg_printf("? info removed: use setter cmds to tune\n");
    }
    else {
        dbg_printf("? unknown cmd: %s\n", cmd);
    }
}

/* 低优先级 IO 任务:20ms 一轮。发一帧 VOFA + 读串口命令。
 * 控制环不依赖本任务,本任务阻塞不影响控制。 */
void dbg_io_task(void *arg)
{
    uint8_t rx[32];
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(20));

        vofa_slot_t slot;
        if (dbg_vofa_pop(&slot)) {
            dbg_vofa_send(slot.data, slot.count);
        }

        int n = dbg_read(rx, sizeof(rx));
        if (n > 0) dbg_process_cmd(rx, n);
    }
}
