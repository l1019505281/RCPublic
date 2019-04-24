#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx.h"

void encoder_TIM4_init(void);
void encoder_TIM8_init(void);
u32 encoder_get_encoder_CNT(u8 TIMx);

#endif
