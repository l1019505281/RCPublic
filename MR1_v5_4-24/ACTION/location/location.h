#ifndef __LOCATION_H
#define __LOCATION_H
#include "stm32f4xx.h"

// 机器人的真实位置
typedef struct ROBOT_REAL_POS{
  float  X_POS;
  float  Y_POS;     
  float  YAW_POS;
	float  LAST_X_POS;
	float  LAST_Y_POS;     
  float  LAST_YAW_POS;
}ROBOT_REAL_POS;

// 机器人位置偏置量
typedef struct ROBOT_POS_OFFSET{
  float  X_CS_OFFSET;
  float  Y_CS_OFFSET;
	float  YAW_CS_OFFSET;
  float  YAW_IN_OFFSET;
}ROBOT_POS_OFFSET;

extern struct GL_POS GL_POS_DATA;

void location_update_robot_real_position(void);

#endif
