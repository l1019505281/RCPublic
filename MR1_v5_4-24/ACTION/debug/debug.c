#include "main.h"

extern float RANGING_DISTANCE;
// 利用串口debug
void debug_by_UART(void)
{
	// debug定位功能
	#if DEBUG_LOCATION
	printf("[LOCALTION]: X: %fmm Y:%fmm YAW: %f° \n",
	ROBOT_REAL_POS_DATA.X_POS,
	ROBOT_REAL_POS_DATA.Y_POS,
	ROBOT_REAL_POS_DATA.YAW_POS);
	
	printf("[ADC]: value: %fmm \n", RANGING_DISTANCE);
	#endif
	
	// debug单个m3508电机（可修改不同的序号）
	#if DEBUG_SINGLE_M3508
	printf("[M3508]: ANGLE: %d RPM:%d CURRENT: %d \n",
	M3508_MOTOR_REAL_INFO.ANGLE,
	M3508_MOTOR_REAL_INFO.RPM,
	M3508_MOTOR_REAL_INFO.CURRENT);
	#endif
	
	// debug路径跟踪
	#if DEBUG_PATH_TRACKING
	printf("[PATH_POINT_COUNTER]: %d \n", PATH_POINT_COUNTER);
  //**********************************************************

	#endif
}


// debug单个m3508电机（可修改不同的序号）
// 用法：将函数单独置于main函数中
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


// 在没有遥控器控制的情况下debug底盘的移动
// 用法：将函数单独置于main函数中
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


// 在单独一条路径上dubug路径跟踪功能
// 用法：正常开启main_loop，将函数置于task函数，需改变NOW_PATH的指向
void debug_path_tracking_with_single_path_in_task(void)
{
	PATH_POINT_SUM = (u32)(sizeof(DEBUG_PLANNING_PATH)/sizeof(DEBUG_PLANNING_PATH[0]));  // 计算路径离散点的总数
	
	if(MOVING_STATE != MOVING_ARRIVED)  // 若还未到达
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







