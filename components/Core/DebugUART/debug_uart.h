#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include "stdint.h"

/**
 * @brief 初始化调试串口(UART0, TX=GPIO43/RX=GPIO44, 115200 8N1)
 *        独立输出到物理 UART 引脚,不依赖 USB console。
 *        同时创建低优先级 IO 任务(20ms 轮询):
 *        发送 VOFA 帧 + 解析串口命令,串口操作不阻塞控制环。
 */
void dbg_uart_init(void);

/**
 * @brief 直接写字符串到调试串口(非阻塞,不经过 console/printf)
 *        注意:低优先级任务调用安全;高优先级实时任务慎用(可能阻塞)。
 */
void dbg_write(const char *s);

/**
 * @brief 格式化输出到调试串口(内部 vsnprintf,栈缓冲 128 字节)
 */
void dbg_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/**
 * @brief VOFA 帧入队(FireWater 协议:浮点逗号分隔+换行帧尾)
 *        非阻塞环形缓冲:控制环等实时任务调用,由 IO 任务异步发送。
 *        环满时静默丢帧(消费端 20ms 一轮,正常速率不会满)。
 */
void dbg_vofa_push(const float *data, int count);

/**
 * @brief 接收函数(非阻塞):检查接收缓冲中可读字节数
 */
int  dbg_available(void);

/**
 * @brief 接收函数(非阻塞):读取数据,返回实际读到的字节数(0=没数据)
 */
int  dbg_read(uint8_t *buf, int max_len);

/**
 * @brief 解析串口命令并在线修改 PID 参数(由 dbg IO 任务调用)
 *        格式: "akp 25\n" / "akd 1.5\n" / "skp 0.2\n" / "pos 100 -100\n"
 */
void dbg_process_cmd(const uint8_t *data, int len);

/* IO 任务入口(debug_uart.c 内部使用) */
void dbg_io_task(void *arg);

#endif
