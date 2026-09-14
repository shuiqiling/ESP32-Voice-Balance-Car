#ifndef MYHAL_MIC_H
#define MYHAL_MIC_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define MIC_SAMPLE_RATE   16000
#define MIC_SAMPLES       1600   // 100ms @16kHz

void    MYHAL_MIC_Init(void);
int     MYHAL_MIC_Take(int16_t *out, int max_samples);
/* 线程安全(tx_mutex),监听回放与 TTS 播报可并发调用 */
void    MYHAL_MIC_Play(const int16_t *data, int samples);

/* 本地监听:把送 ASR 的音频同步回放,默认关闭,调试麦克风通路时打开 */
void    MYHAL_MIC_SetMonitor(bool enabled);
bool    MYHAL_MIC_GetMonitor(void);

#endif
