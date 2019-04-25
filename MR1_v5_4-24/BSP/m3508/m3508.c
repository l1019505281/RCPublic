#include "main.h"


/*********************
 velocity 机器人的速度 
 speed    电机轴rpm 
 RPM      电机转子rpm
 *********************/


M3508_REAL_INFO M3508_MOTOR_REAL_INFO;
PID M3508_PID;


// M3508初始化
void m3508_m3508_motor_init(void)
{
	// PID初始化
	PID_parameter_init(&M3508_PID, 10.0, 1.0, 0.0, 15000.0, 15000.0);
}


// 利用电机通过CAN反馈的数据更新m3508的状态信息
void m3508_update_m3508_info(CanRxMsg *msg)
{
	switch(msg -> StdId)  // 检测标准ID
	{
		case M3508_MOTOR_ID:
		{ 
			M3508_MOTOR_REAL_INFO.ANGLE = (msg -> Data[0] << 8) | msg -> Data[1];    // 转子机械角度
			M3508_MOTOR_REAL_INFO.RPM = (msg -> Data[2] << 8) | msg -> Data[3];      // 实际转子转速
			M3508_MOTOR_REAL_INFO.CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
		}; break;
		
		default: break;
	}
}


// 通过CAN2发送底盘m3508的电流
// 接受频率：1kHz
// 转子角度范围值：0-8191（对应0到360度）
// 转子转速单位为RPM
// 电机温度单位为摄氏度
void m3508_send_m3508_currents(int16_t currents, u16 motor_id)
{
	CanTxMsg tx_message;

	// 配置控制段
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 0x08;
	
	// 配置仲裁段和数据段
	switch(motor_id)
	{
		case 0x201:
		{		
			tx_message.StdId = 0x200;  // ID为 1 2 3 4 的电机
			tx_message.Data[0] = (uint8_t)(currents >> 8);
			tx_message.Data[1] = (uint8_t)currents;
		}
		break;
		
		case 0x202:
		{		
			tx_message.StdId = 0x200;  // ID为 1 2 3 4 的电机
			tx_message.Data[2] = (uint8_t)(currents >> 8);
			tx_message.Data[3] = (uint8_t)currents;
		}
		break;
		
		case 0x203:
		{		
			tx_message.StdId = 0x200;  // ID为 1 2 3 4 的电机
			tx_message.Data[4] = (uint8_t)(currents >> 8);
			tx_message.Data[5] = (uint8_t)currents;
		}
		break;
		
		case 0x204:
		{		
			tx_message.StdId = 0x200;  // ID为 1 2 3 4 的电机
			tx_message.Data[6] = (uint8_t)(currents >> 8);
			tx_message.Data[7] = (uint8_t)currents;
		}
		break;

		case 0x205:
		{		
			tx_message.StdId = 0x1ff;  // ID为 5 6 7 8 的电机
			tx_message.Data[0] = (uint8_t)(currents >> 8);
			tx_message.Data[1] = (uint8_t)currents;
		}
		break;

		case 0x206:
		{		
			tx_message.StdId = 0x1ff;  // ID为 5 6 7 8 的电机
			tx_message.Data[2] = (uint8_t)(currents >> 8);
			tx_message.Data[3] = (uint8_t)currents;
		}
		break;

		case 0x207:
		{		
			tx_message.StdId = 0x1ff;  // ID为 5 6 7 8 的电机
			tx_message.Data[4] = (uint8_t)(currents >> 8);
			tx_message.Data[5] = (uint8_t)currents;
		}
		break;

		case 0x208:
		{		
			tx_message.StdId = 0x1ff;   // ID为 5 6 7 8 的电机
			tx_message.Data[6] = (uint8_t)(currents >> 8);
			tx_message.Data[7] = (uint8_t)currents;
		}
		break;		
		default: break;
	}
	
	CAN_Transmit(CAN2, &tx_message);
}


/*WAITING_TEST*/
// 通过PID设置底盘m3508的转子转速
// 设置速度，因为需要调节PID的原因，不能只设一次，应该不断循环设置
void m3508_set_m3508_rpm(int motor_rpm, u16 motor_id)  // 强制转换为整型
{
	// 增量式PID计算
	PID_incremental_PID_calculation(&M3508_PID, M3508_MOTOR_REAL_INFO.RPM, motor_rpm);
	
	// 发送经PID调节后的电流
	m3508_send_m3508_currents(M3508_PID.output, motor_id);
}



