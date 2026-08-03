#ifndef MYHAL_MIC_H
#define MYHAL_MIC_H

#include <stdint.h>
#include <stddef.h>

#define MIC_SAMPLE_RATE   16000
#define MIC_SAMPLES       1600   // 100ms @16kHz

void    MYHAL_MIC_Init(void);
int     MYHAL_MIC_Take(int16_t *out, int max_samples);

#endif
