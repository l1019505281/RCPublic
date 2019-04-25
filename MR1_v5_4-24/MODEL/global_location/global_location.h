#ifndef __GLOBAL_LOCATION_H
#define __GLOBAL_LOCATION_H
#include "stm32f4xx.h"

#define WHEEL_DIAMETER 50.8f  // ����ֱ��
#define ENCODER_RESIOLUTION 1000  // �������ֱ���
#define DISTANCE_PER_CNT (PI*WHEEL_DIAMETER/(ENCODER_RESIOLUTION*4))  // ÿ��CNT�����·��ֵ
#define SIN45 0.70710678f 

#define Lx 13.5411f  // ˳ʱ�������Ӵ�ֱ�������ľ��루��λ��mm��
#define Ly 13.5411f  // ˳ʱ��β���Ӵ�ֱ�������ľ��루��λ��mm��
#define YAW0 45  // ˳ʱ������������������ϵ�ĽǶȣ���Χ(-180, 180]��

// �������н��ľ�������
typedef struct ENCO_DIS
{
	int CNT_X;
	int CNT_Y;
	float DELTA_DIS_X;
	float DELTA_DIS_Y;
}ENCO_DIS;

// ȫ����λģ�鶨λ��λ��
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
