#ifndef SER_BALANCECAR_H
#define SER_BALANCECAR_H

#include "stdint.h"

void SER_BALANCECAR_Init(void);
void SER_BALANCECAR_STAND(float GOATANGLE_R,float GOATANGLE_L);
void SER_BALANCECAR_SPEED(float RefenceValue_R,float RefenceValue_L);
void SER_BALANCECAR_POSITION(float RefenceValue_R,float RefenceValue_L);
void SER_BALANCECAR_Task(void *arg);
#endif
