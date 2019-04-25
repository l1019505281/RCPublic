#ifndef __TIM2_H
#define __TIM2_H

#include "stm32f4xx.h"

typedef struct TIM2_TASK_COUNTER
{
	u32 COUNTER_IT;
	u8  COUNTER_1MS;
	u8  COUNTER_2MS;
	u8  COUNTER_5MS;
	u8  COUNTER_10MS;
	u8  COUNTER_20MS;
	u8  COUNTER_50MS;
	u8  COUNTER_100MS;
	u8  COUNTER_200MS;
	u32 COUNTER_500MS;
	u32 COUNTER_1000MS;
}TIM2_TASK_COUNTER;

void tim2_TIM2_init(void);
void tim2_start_main_loop(void);

#endif
