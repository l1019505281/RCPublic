#ifndef __BSP_H
#define __BSP_H

#include "stm32f4xx.h"

// 预充电时间
#define PRE_CHARGE_TIME 500  // 单位：ms

void bsp_pre_charge(void);

#endif
