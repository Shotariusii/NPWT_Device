#ifndef Delay_uS_H
#define Delay_uS_H
#include "main.h"

#define DWT_DELAY_NEWBIE 0

void DWT_Init(void);
void DWT_Delay(uint32_t us);

#endif
