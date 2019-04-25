#ifndef __LOCATION_CORRECTION_H
#define __LOCATION_CORRECTION_H

#include "stm32f4xx.h"

void lc_color_sensor_location_correction(void);
void lc_color_MOVING_URTUU_TO_RZ(void);
void lc_color_laser_ranging_location_correction(void);

// 竖直线的X轴坐标
#define VERTICAL_INSIDE_LINE_X -1955
#define VERTICAL_OUTSIDE_LINE_X -525
#define VERTICAL_CENTER_LINE_X -1240

// 水平线的Y轴坐标
#define ACLINIC_LINE_1_Y 1265
#define ACLINIC_LINE_2_Y 2015
#define ACLINIC_LINE_3_Y 2765
#define ACLINIC_LINE_4_Y 3515
#define ACLINIC_LINE_BRIDGE_Y 4265

// 竖直色标传感器到底盘中心的距离（正值）
#define DISTANCE_CENTER_TO_VERTICAL_SENSOR 165

// 水平色标传感器到底盘中心的距离（正值）
#define DISTANCE_CENTER_TO_ACLINIC_SENSOR 60

#endif 
