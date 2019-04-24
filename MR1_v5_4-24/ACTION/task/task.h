#ifndef __TASK_H
#define __TASK_H
#include "stm32f4xx.h"

extern struct TIM2_TASK_COUNTER TIM2_TASK_COUNTER_DATA;

void task_frequency_control_main_loop(void);
void task_1000Hz(void);
void task_500Hz(void);
void task_200Hz(void);
void task_100Hz(void);
void task_50Hz(void);
void task_20Hz(void);
void task_10Hz(void);
void task_5Hz(void);
void task_2Hz(void);
void task_1Hz(void);

#endif
