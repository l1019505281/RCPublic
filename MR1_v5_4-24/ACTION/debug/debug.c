#include "main.h"

extern float RANGING_DISTANCE;
// ���ô���debug
void debug_by_UART(void)
{
	// debug��λ����
	#if DEBUG_LOCATION
	printf("[LOCALTION]: X: %fmm Y:%fmm YAW: %f�� \n",
	ROBOT_REAL_POS_DATA.X_POS,
	ROBOT_REAL_POS_DATA.Y_POS,
	ROBOT_REAL_POS_DATA.YAW_POS);
	
	printf("[ADC]: value: %fmm \n", RANGING_DISTANCE);
	#endif
	
	// debug����m3508��������޸Ĳ�ͬ����ţ�
	#if DEBUG_SINGLE_M3508
	printf("[M3508]: ANGLE: %d RPM:%d CURRENT: %d \n",
	M3508_MOTOR_REAL_INFO.ANGLE,
	M3508_MOTOR_REAL_INFO.RPM,
	M3508_MOTOR_REAL_INFO.CURRENT);
	#endif
	
	// debug·������
	#if DEBUG_PATH_TRACKING
	printf("[PATH_POINT_COUNTER]: %d \n", PATH_POINT_COUNTER);
  //**********************************************************

	#endif
}


// debug����m3508��������޸Ĳ�ͬ����ţ�
// �÷�����������������main������
void debug_single_m3508_in_main(void)
{
	int16_t m3508_rpm = 100;
	
	delay_init(168);
	uart_init(115200);
	can2_CAN2_init();
	
	m3508_m3508_motor_init();
	
	while(1)
	{	
		delay_ms(1);
		
		PID_incremental_PID_calculation(&M3508_PID, M3508_MOTOR_REAL_INFO.RPM, m3508_rpm);
		m3508_send_m3508_currents(M3508_PID.output, M3508_MOTOR_ID);
		
		debug_by_UART();
	}
}


// ��û��ң�������Ƶ������debug���̵��ƶ�
// �÷�����������������main������
void debug_chassis_without_remote_control_in_main(void)
{
	extern struct ROBOT_TARGET_VELOCITY ROBOT_TARGET_VELOCITY_DATA;
	
	delay_init(168);
	uart_init(115200);
	can1_CAN1_init();
	cd_chassis_driver_init();
		
	ROBOT_TARGET_VELOCITY_DATA.Vx_RPM = 0;
	ROBOT_TARGET_VELOCITY_DATA.Vy_RPM = 1500;
	ROBOT_TARGET_VELOCITY_DATA.W_RPM = 0;
	
	while(1)
	{
		delay_ms(1);
		
		cd_chassis_velocity_adjust();
		
		debug_by_UART();
	}
}


// �ڵ���һ��·����dubug·�����ٹ���
// �÷�����������main_loop������������task��������ı�NOW_PATH��ָ��
void debug_path_tracking_with_single_path_in_task(void)
{
	PATH_POINT_SUM = (u32)(sizeof(DEBUG_PLANNING_PATH)/sizeof(DEBUG_PLANNING_PATH[0]));  // ����·����ɢ�������
	
	if(MOVING_STATE != MOVING_ARRIVED)  // ����δ����
	{
		moving_path_tracking();
	}
	else
	{
		ROBOT_TARGET_VELOCITY_DATA.Vx_RPM = 0;
		ROBOT_TARGET_VELOCITY_DATA.Vy_RPM = 0;
		ROBOT_TARGET_VELOCITY_DATA.W_RPM = 0;
	}
}







