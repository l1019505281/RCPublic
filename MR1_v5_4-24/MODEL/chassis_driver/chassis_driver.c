#include "main.h"


CHASSIS_MOTOR_RPM CHASSIS_MOTOR_RPM_DATA;
ROBOT_TARGET_VELOCITY ROBOT_TARGET_VELOCITY_DATA;


// 初始化底盘电机配置
void cd_chassis_driver_init(void)
{
	delay_ms(100);
	CAN_RoboModule_DRV_Reset(0, 0);  // 对0组驱动器进行复位
	delay_ms(500);                                     
	CAN_RoboModule_DRV_Config(0, 0, 1, 0);  // 配置为1ms传回一次数据 关闭左右限位功能
	delay_ms(50); 
	CAN_RoboModule_DRV_Mode_Choice(0, 0, Velocity_Mode);  // 选择进入速度模式
	delay_ms(500);
}


// 4轮世界坐标系逆运动学
// theta为机器人坐标系x轴与世界坐标系x轴夹角 单位：度
// W：正值-逆时针 负值-顺时针
void cd_world_kinematic_conversion_4wheels(float Vx_RPM, float Vy_RPM, float W_RPM, float theta)
{
	theta = PI * theta / 180.0f;
	CHASSIS_MOTOR_RPM_DATA.MOTOR1_RPM =  cos(theta+PI/4.0f) * Vx_RPM + sin(theta+PI/4.0f) * Vy_RPM - W_RPM;
	CHASSIS_MOTOR_RPM_DATA.MOTOR2_RPM =  cos(theta-PI/4.0f) * Vx_RPM + sin(theta-PI/4.0f) * Vy_RPM + W_RPM;
	CHASSIS_MOTOR_RPM_DATA.MOTOR3_RPM = -cos(theta+PI/4.0f) * Vx_RPM - sin(theta+PI/4.0f) * Vy_RPM - W_RPM;
	CHASSIS_MOTOR_RPM_DATA.MOTOR4_RPM = -cos(theta-PI/4.0f) * Vx_RPM - sin(theta-PI/4.0f) * Vy_RPM + W_RPM;
}


// 控制底盘跟着速度期望值调整速度
void cd_chassis_velocity_adjust(void)
{
	cd_world_kinematic_conversion_4wheels(ROBOT_TARGET_VELOCITY_DATA.Vx_RPM,
																				ROBOT_TARGET_VELOCITY_DATA.Vy_RPM, 
																				ROBOT_TARGET_VELOCITY_DATA.W_RPM,
																				ROBOT_REAL_POS_DATA.YAW_POS);

	// 按期望值调整速度
	// 因为电机接线原因进行转动方向调整
	CHASSIS_MOTOR_RPM_DATA.MOTOR1_RPM = -CHASSIS_MOTOR_RPM_DATA.MOTOR1_RPM;
	CHASSIS_MOTOR_RPM_DATA.MOTOR2_RPM = -CHASSIS_MOTOR_RPM_DATA.MOTOR2_RPM;
	CHASSIS_MOTOR_RPM_DATA.MOTOR3_RPM =  CHASSIS_MOTOR_RPM_DATA.MOTOR3_RPM;
	CHASSIS_MOTOR_RPM_DATA.MOTOR4_RPM = -CHASSIS_MOTOR_RPM_DATA.MOTOR4_RPM;
	
	CAN_RoboModule_DRV_Velocity_Mode(0, 1, 5000, CHASSIS_MOTOR_RPM_DATA.MOTOR1_RPM);  // 因为驱动器ID不同有所改动
	CAN_RoboModule_DRV_Velocity_Mode(0, 2, 5000, CHASSIS_MOTOR_RPM_DATA.MOTOR2_RPM);
	CAN_RoboModule_DRV_Velocity_Mode(0, 3, 5000, CHASSIS_MOTOR_RPM_DATA.MOTOR3_RPM);
	CAN_RoboModule_DRV_Velocity_Mode(0, 4, 5000, CHASSIS_MOTOR_RPM_DATA.MOTOR4_RPM);
}
