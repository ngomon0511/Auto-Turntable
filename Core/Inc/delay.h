#ifndef NHC_DELAY_H_
#define NHC_DELAY_H_

#include "stm32f1xx_hal.h"

void Delay_Init(void);
void Delay_Ms(uint32_t u32Delay);
void Delay_Us(uint32_t u32Delay);

#endif
