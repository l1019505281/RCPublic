#ifndef __ADC_H
#define __ADC_H

#include "stm32f4xx.h"
#include "queue.h"




void adc_adc3_init(void);
void adc_update_ranging_info(u16 value);
void adc_update_info(void);

#endif
