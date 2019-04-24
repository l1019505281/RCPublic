#ifndef __GLOBAL_LOCATION_H
#define __GLOBAL_LOCATION_H
#include "stm32f4xx.h"

#define WHEEL_DIAMETER 50.8f  // 轮子直径
#define ENCODER_RESIOLUTION 1000  // 编码器分辨率
#define DISTANCE_PER_CNT (PI*WHEEL_DIAMETER/(ENCODER_RESIOLUTION*4))  // 每个CNT代表的路程值
#define SIN45 0.70710678f 

#define Lx 13.5411f  // 顺时针首轮子垂直底盘中心距离（单位：mm）
#define Ly 13.5411f  // 顺时针尾轮子垂直底盘中心距离（单位：mm）
#define YAW0 45  // 顺时针首轮子离世界坐标系的角度（范围(-180, 180]）

// 编码器行进的距离数据
typedef struct ENCO_DIS
{
	int CNT_X;
	int CNT_Y;
	float DELTA_DIS_X;
	float DELTA_DIS_Y;
}ENCO_DIS;

// 全场定位模块定位的位置
typedef struct GL_POS
{
	float X;
	float Y;
	float YAW;
}GL_POS;

extern struct SAngle GYRO_ANGLE;
extern struct ROBOT_POS_OFFSET ROBOT_POS_OFFSET_DATA;
extern struct ENCO_DIS ENCO_DIS_DATA;

void gl_GL_model_init(void);
void gl_update_GL_position(void);
void gl_encoder_to_GL_conversion(void);

#endif
