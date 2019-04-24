#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H
#include "stm32f4xx.h"

// 底盘电机RPM
typedef struct CHASSIS_MOTOR_RPM
{
	int16_t MOTOR1_RPM;
	int16_t MOTOR2_RPM;
	int16_t MOTOR3_RPM;
	int16_t MOTOR4_RPM;
}CHASSIS_MOTOR_RPM;

// 底盘期望速度
typedef struct ROBOT_TARGET_VELOCITY
{
	float Vx;
	float Vy;
	float W;
	int16_t Vx_RPM;
	int16_t Vy_RPM;
	int16_t W_RPM;
}ROBOT_TARGET_VELOCITY;

extern struct ROBOT_REAL_POS ROBOT_REAL_POS_DATA;

void cd_chassis_driver_init(void);
void cd_robot_kinematic_conversion_3wheels(float Vx, float Vy, float Vz);
void cd_world_kinematic_conversion_3wheels(float Vx, float Vy, float W, float theta);
void cd_world_kinematic_conversion_4wheels(float Vx, float Vy, float W, float theta);
void cd_chassis_velocity_adjust(void);

#endif
