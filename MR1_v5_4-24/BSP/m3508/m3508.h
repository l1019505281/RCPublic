#ifndef __M3508_H
#define __M3508_H
#include "stm32f4xx.h"

#define M3508_MOTOR_ID 0x201

// M3508返回的电机真实信息
typedef struct M3508_REAL_INFO
{
	uint16_t ANGLE;   								
	int16_t RPM;								
	int16_t CURRENT; 
}M3508_REAL_INFO;

void m3508_m3508_motor_init(void);
void m3508_update_m3508_info(CanRxMsg *msg);
void m3508_send_m3508_currents(int16_t currents, u16 motor_id);
void m3508_set_m3508_rpm(int motor_rpm, u16 motor_id);

#endif
