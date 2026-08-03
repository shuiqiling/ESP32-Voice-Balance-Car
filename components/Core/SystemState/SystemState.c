#include "SystemState.h"
#include <stdbool.h>

volatile PermitLock_t AITRANSCJSON_PermitLock;
volatile PermitLock_t ASRSENDDATACJSON_PermitLock;

volatile PermitLock_t WIFIGOTIP_PermitLock;

volatile float ser_pos_target_r = 0;   /* 串口位置控制:右轮目标(mm) */
volatile float ser_pos_target_l = 0;   /* 左轮目标(mm) */

volatile bool ser_emg_stop = false;   /* 急停标志 */
