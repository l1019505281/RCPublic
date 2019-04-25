#ifndef __LOCATION_CORRECTION_H
#define __LOCATION_CORRECTION_H

#include "stm32f4xx.h"

void lc_color_sensor_location_correction(void);
void lc_color_MOVING_URTUU_TO_RZ(void);
void lc_color_laser_ranging_location_correction(void);

// ��ֱ�ߵ�X������
#define VERTICAL_INSIDE_LINE_X -1955
#define VERTICAL_OUTSIDE_LINE_X -525
#define VERTICAL_CENTER_LINE_X -1240

// ˮƽ�ߵ�Y������
#define ACLINIC_LINE_1_Y 1265
#define ACLINIC_LINE_2_Y 2015
#define ACLINIC_LINE_3_Y 2765
#define ACLINIC_LINE_4_Y 3515
#define ACLINIC_LINE_BRIDGE_Y 4265

// ��ֱɫ�괫�������������ĵľ��루��ֵ��
#define DISTANCE_CENTER_TO_VERTICAL_SENSOR 165

// ˮƽɫ�괫�������������ĵľ��루��ֵ��
#define DISTANCE_CENTER_TO_ACLINIC_SENSOR 60

#endif 
