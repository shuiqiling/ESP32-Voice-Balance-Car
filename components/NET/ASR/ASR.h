#ifndef ASR_H
#define ASR_H

#include <stddef.h>
#include <stdint.h>

void ASR_Init(void);
void ASR_Start(void);
void ASR_Stop(void);
void asr_send_task(void *arg);

#endif
