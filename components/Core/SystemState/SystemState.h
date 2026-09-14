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

/* 串口目标位置(mm),串口命令任务写,控制任务读并消费。
 * 两个目标值和 ser_pos_cmd_pending 必须整体原子访问 —— dbg_io 任务没绑核、
 * 控制任务在 core 1,两者可能真并行。用 ser_pos_mux 保护这三者的读写。*/
extern volatile float ser_pos_target_r;
extern volatile float ser_pos_target_l;
/* 有新位置命令待处理:dbg_io 任务置 true,控制任务取走后置 false。
 * 单靠比较目标值无法判断"新命令"(目标可能恰好等于当前值),所以用显式标志。*/
extern volatile bool ser_pos_cmd_pending;

/* 保护上面三个变量的自旋锁(跨核,不能用临界区宏之外的方式绕过) */
extern portMUX_TYPE ser_pos_mux;

/* 急停标志:AI 任务写(true=急停),控制任务读并消费清零 */
extern volatile bool ser_emg_stop;

/* TTS 正在出声:tts_play_task 置位/清零,asr_send_task 读。
 * 喇叭和麦克风在同一块板上,播报的声音会被自己的麦克风拾到并原样送到
 * 阿里云 ASR,识别成一条新指令 → 再触发一次播报,形成自问自答的死循环。
 * 固件里没有 AEC,唯一能断环的办法就是播报期间停止上传麦克风数据。*/
extern volatile bool tts_speaking;

#endif
