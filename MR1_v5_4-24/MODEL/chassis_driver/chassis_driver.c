#include "main.h"


CHASSIS_MOTOR_RPM CHASSIS_MOTOR_RPM_DATA;
ROBOT_TARGET_VELOCITY ROBOT_TARGET_VELOCITY_DATA;


// ��ʼ�����̵������
void cd_chassis_driver_init(void)
{
	delay_ms(100);
	CAN_RoboModule_DRV_Reset(0, 0);  // ��0�����������и�λ
	delay_ms(500);                                     
	CAN_RoboModule_DRV_Config(0, 0, 1, 0);  // ����Ϊ1ms����һ������ �ر�������λ����
	delay_ms(50); 
	CAN_RoboModule_DRV_Mode_Choice(0, 0, Velocity_Mode);  // ѡ������ٶ�ģʽ
	delay_ms(500);
}


// 4����������ϵ���˶�ѧ
// thetaΪ����������ϵx������������ϵx��н� ��λ����
// W����ֵ-��ʱ�� ��ֵ-˳ʱ��
void cd_world_kinematic_conversion_4wheels(float Vx_RPM, float Vy_RPM, float W_RPM, float theta)
{
	theta = PI * theta / 180.0f;
	CHASSIS_MOTOR_RPM_DATA.MOTOR1_RPM =  cos(theta+PI/4.0f) * Vx_RPM + sin(theta+PI/4.0f) * Vy_RPM - W_RPM;
	CHASSIS_MOTOR_RPM_DATA.MOTOR2_RPM =  cos(theta-PI/4.0f) * Vx_RPM + sin(theta-PI/4.0f) * Vy_RPM + W_RPM;
	CHASSIS_MOTOR_RPM_DATA.MOTOR3_RPM = -cos(theta+PI/4.0f) * Vx_RPM - sin(theta+PI/4.0f) * Vy_RPM - W_RPM;
	CHASSIS_MOTOR_RPM_DATA.MOTOR4_RPM = -cos(theta-PI/4.0f) * Vx_RPM - sin(theta-PI/4.0f) * Vy_RPM + W_RPM;
}


// ���Ƶ��̸����ٶ�����ֵ�����ٶ�
void cd_chassis_velocity_adjust(void)
{
	cd_world_kinematic_conversion_4wheels(ROBOT_TARGET_VELOCITY_DATA.Vx_RPM,
																				ROBOT_TARGET_VELOCITY_DATA.Vy_RPM, 
																				ROBOT_TARGET_VELOCITY_DATA.W_RPM,
																				ROBOT_REAL_POS_DATA.YAW_POS);

	// ������ֵ�����ٶ�
	// ��Ϊ�������ԭ�����ת���������
	CHASSIS_MOTOR_RPM_DATA.MOTOR1_RPM = -CHASSIS_MOTOR_RPM_DATA.MOTOR1_RPM;
	CHASSIS_MOTOR_RPM_DATA.MOTOR2_RPM = -CHASSIS_MOTOR_RPM_DATA.MOTOR2_RPM;
	CHASSIS_MOTOR_RPM_DATA.MOTOR3_RPM =  CHASSIS_MOTOR_RPM_DATA.MOTOR3_RPM;
	CHASSIS_MOTOR_RPM_DATA.MOTOR4_RPM = -CHASSIS_MOTOR_RPM_DATA.MOTOR4_RPM;
	
	CAN_RoboModule_DRV_Velocity_Mode(0, 1, 5000, CHASSIS_MOTOR_RPM_DATA.MOTOR1_RPM);  // ��Ϊ������ID��ͬ�����Ķ�
	CAN_RoboModule_DRV_Velocity_Mode(0, 2, 5000, CHASSIS_MOTOR_RPM_DATA.MOTOR2_RPM);
	CAN_RoboModule_DRV_Velocity_Mode(0, 3, 5000, CHASSIS_MOTOR_RPM_DATA.MOTOR3_RPM);
	CAN_RoboModule_DRV_Velocity_Mode(0, 4, 5000, CHASSIS_MOTOR_RPM_DATA.MOTOR4_RPM);
}
