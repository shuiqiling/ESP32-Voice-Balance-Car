#ifndef SYSTEMSTATE_H
#define SYSTEMSTATE_H

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum{
    PermitLock_Lock,   /* 0:锁定(默认值,全局变量零初始化即锁定) */
    PermitLock_Free    /* 1:解锁 */
}PermitLock_t;

/* ===== AI 队列消息类型(跨 NET / SER 组件共享) ===== */
typedef struct {
    char cmd[64];
} asr_result_t;

extern QueueHandle_t asr_queue;   /* ASR 识别结果队列 */

typedef struct {
    char buf[64];
} ai_voice_t;

extern QueueHandle_t ai_voice_queue;   /* AI 语音回复队列 */

typedef struct {
    char cmd[64];
    int16_t cmd_value;
} ai_cmd_t;

extern QueueHandle_t ai_cmd_queue;   /* AI 控制指令队列 */

/* ===== 跨任务共享标志 =====
 * volatile + 32 位对齐读写(xtensa 上为单指令原子),保证跨核读写不经过
 * 寄存器缓存;写入方与读取方所在任务见各变量注释。 */

extern volatile PermitLock_t AITRANSCJSON_PermitLock;   /* http 事件回调写, ai_task 读 */
extern volatile PermitLock_t ASRSENDDATACJSON_PermitLock;/* websocket 回调写, asr_send_task 读 */
extern volatile PermitLock_t WIFIGOTIP_PermitLock;       /* wifi 事件回调写, ai_task 读 */

/* 串口目标位置(mm),串口命令任务写,控制任务读 */
extern volatile float ser_pos_target_r;
extern volatile float ser_pos_target_l;

/* 急停标志:AI 任务写(true=急停),控制任务读并消费清零 */
extern volatile bool ser_emg_stop;

#endif
