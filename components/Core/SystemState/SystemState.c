#include "SystemState.h"
#include <stdbool.h>

volatile PermitLock_t AITRANSCJSON_PermitLock;
volatile PermitLock_t ASRSENDDATACJSON_PermitLock;

volatile PermitLock_t WIFIGOTIP_PermitLock;

volatile float ser_pos_target_r = 0;   /* 串口位置控制:右轮目标(mm) */
volatile float ser_pos_target_l = 0;   /* 左轮位置控制:左轮目标(mm) */
volatile bool  ser_pos_cmd_pending = false;  /* 有未消费的串口位置命令 */

volatile bool ser_emg_stop = false;   /* 急停标志 */

volatile bool tts_speaking = false;   /* TTS 正在出声,期间停止上传麦克风 */

portMUX_TYPE ser_pos_mux = portMUX_INITIALIZER_UNLOCKED;

